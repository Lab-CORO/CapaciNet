#!/usr/bin/env python3
"""Evaluate the mean reachability index of the workspace during a grasping trajectory.

Builds a reachability map (RM) from every /curobo_trajectory_planner/voxel_grid_sparse
message (own ReachabilityEngine instance, same as brain.py). Every time a new predicted
trajectory arrives on /mpc_predicted_path, scores two spherical workspaces against the
latest RM:

  - Q_goal: sphere centered on the current /curobo_trajectory_planner/mpc_goal
  - Q_path: mean over spheres centered on (a subsample of) the path poses

Per-sample values are published, logged and appended to a CSV. Samples are grouped into
"episodes" (one grasping trajectory): an episode ends when no path message has been seen
for `episode_timeout` seconds, at which point a summary row is logged/published/written.
"""

import csv
from datetime import datetime
import os
import time

from curobo_msgs.msg import SparseVoxelGrid
from geometry_msgs.msg import Pose, PoseStamped
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32, String
from tf2_geometry_msgs import do_transform_pose_stamped
import tf2_ros
from tf2_ros import Buffer, TransformListener

from capacitynet.rm_model.reachability_engine import ReachabilityEngine
from capacitynet.control.scripts.workspace_evaluation import WorkspaceEvaluation

# Headless-safe backend switch (the node has no display); done after the
# import block so it stays a normal statement rather than splitting the
# imports and tripping E402.
plt.switch_backend('Agg')

_SAMPLE_CSV_FIELDS = [
    'timestamp', 'episode_id', 'seq', 'goal_x', 'goal_y', 'goal_z', 'path_frame',
    'n_poses', 'n_eval', 'q_goal', 'q_path_mean', 'q_path_min', 'q_path_max',
    'rm_age_ms', 'compute_ms',
]

_EPISODE_CSV_FIELDS = [
    'episode_id', 't_start', 't_end', 'duration_s', 'n_samples', 'rate_hz',
    'q_goal_mean', 'q_path_mean', 'q_path_min', 'q_path_max', 'goal_x', 'goal_y', 'goal_z',
]


class _EpisodeAccumulator:
    """Running stats for one grasping trajectory (one stream of path messages)."""

    def __init__(self, episode_id, t_start):
        self.episode_id = episode_id
        self.t_start = t_start
        self.n_samples = 0
        self.q_goal_values = []
        self.q_path_means = []
        self.q_path_min = float('inf')
        self.q_path_max = float('-inf')
        self.last_goal = None

        # Per-sample series (relative to t_start) kept for the end-of-episode plot.
        self.rel_times = []
        self.q_path_min_series = []
        self.q_path_max_series = []

        # (rel_time, state_name) markers from ~/state, kept for the end-of-episode plot.
        self.state_events = []

    def add_state_event(self, t_now, state_name):
        self.state_events.append((t_now - self.t_start, state_name))

    def add_sample(self, t_now, q_goal, q_path_mean, q_path_min, q_path_max, goal):
        self.n_samples += 1
        self.q_goal_values.append(q_goal)
        self.q_path_means.append(q_path_mean)
        self.q_path_min = min(self.q_path_min, q_path_min)
        self.q_path_max = max(self.q_path_max, q_path_max)
        self.last_goal = goal

        self.rel_times.append(t_now - self.t_start)
        self.q_path_min_series.append(q_path_min)
        self.q_path_max_series.append(q_path_max)

    def summarize(self, t_end):
        duration = max(t_end - self.t_start, 1e-9)
        goal = self.last_goal or (0.0, 0.0, 0.0)
        return {
            'episode_id': self.episode_id,
            't_start': self.t_start,
            't_end': t_end,
            'duration_s': duration,
            'n_samples': self.n_samples,
            'rate_hz': self.n_samples / duration,
            'q_goal_mean': float(np.mean(self.q_goal_values)) if self.q_goal_values else 0.0,
            'q_path_mean': float(np.mean(self.q_path_means)) if self.q_path_means else 0.0,
            'q_path_min': self.q_path_min if self.q_path_means else 0.0,
            'q_path_max': self.q_path_max if self.q_path_means else 0.0,
            'goal_x': goal[0], 'goal_y': goal[1], 'goal_z': goal[2],
        }


class WorkspaceReachabilityNode(Node):
    def __init__(self):
        super().__init__('workspace_reachability_node')

        self._load_parameters()
        self._setup_engine()
        self._setup_csv()
        self._setup_interfaces()

        # Latest reachability map + the voxel-grid metadata it was built from.
        self._rm = None
        self._vg_info = None
        self._vg_frame = None
        self._rm_stamp = None

        self._goal = None            # (x, y, z) in self._vg_frame, or None
        self._goal_eval = None       # cached WorkspaceEvaluation for the current goal
        self._goal_eval_center = None

        self._latest_state = None

        self._episode = None
        self._next_episode_id = 0
        self._last_path_time = None
        self._seq = 0

        # Wall timer to flush an episode summary even if the path stream just
        # stops (goal reached / aborted) instead of only flushing on the next message.
        self.create_timer(0.2, self._check_episode_timeout)

        self.get_logger().info('Workspace reachability node initialized')
        self.get_logger().info(f'  - Voxel grid topic: {self.voxel_grid_topic}')
        self.get_logger().info(f'  - Goal topic: {self.goal_topic}')
        self.get_logger().info(f'  - Path topic: {self.path_topic}')
        self.get_logger().info(f'  - State topic: {self.state_topic}')
        self.get_logger().info(f'  - Workspace radius: {self.workspace_radius} m')
        self.get_logger().info(f'  - CSV: {self._samples_csv_path} / {self._episodes_csv_path}')

    # ── Setup ────────────────────────────────────────────────────────────────

    def _load_parameters(self):
        self.declare_parameter(
            'model_config_path', '/home/ros2_ws/src/capacitynet/config/test_reach.yaml')
        self.declare_parameter('fp16', False)

        self.declare_parameter('voxel_grid_topic', '/curobo_trajectory_planner/voxel_grid_sparse')
        self.declare_parameter('goal_topic', '/curobo_trajectory_planner/mpc_goal')
        self.declare_parameter('path_topic', '/mpc_predicted_path_full')
        self.declare_parameter('state_topic', '/object_grasper/state')
        self.declare_parameter('planning_frame', 'dsr01/base_link')

        self.declare_parameter('workspace_radius', 0.30)
        self.declare_parameter('path_max_samples', 16)
        self.declare_parameter('episode_timeout', 1.0)
        self.declare_parameter('episode_goal_jump', 0.0)

        self.declare_parameter('output_csv_prefix', '')
        self.declare_parameter('log_each_sample', True)
        self.declare_parameter('generate_episode_plot', True)

        self.model_config_path = self.get_parameter('model_config_path').value
        self.fp16 = bool(self.get_parameter('fp16').value)

        self.voxel_grid_topic = self.get_parameter('voxel_grid_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.planning_frame = self.get_parameter('planning_frame').value

        self.workspace_radius = float(self.get_parameter('workspace_radius').value)
        self.path_max_samples = int(self.get_parameter('path_max_samples').value)
        self.episode_timeout = float(self.get_parameter('episode_timeout').value)
        self.episode_goal_jump = float(self.get_parameter('episode_goal_jump').value)

        self.log_each_sample = bool(self.get_parameter('log_each_sample').value)
        self.generate_episode_plot = bool(self.get_parameter('generate_episode_plot').value)

        prefix = self.get_parameter('output_csv_prefix').value
        if not prefix:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            prefix = f'~/ros2_ws/src/capacitynet/results/workspace_reach_{ts}'
        self.output_csv_prefix = os.path.expanduser(prefix)

    def _setup_engine(self):
        import torch
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Loading reachability model on {self.device}...')
        self.engine = ReachabilityEngine(self.model_config_path, fp16=self.fp16)
        self.get_logger().info('Reachability model loaded')

    def _setup_csv(self):
        os.makedirs(os.path.dirname(self.output_csv_prefix), exist_ok=True)
        self._samples_csv_path = f'{self.output_csv_prefix}_samples.csv'
        self._episodes_csv_path = f'{self.output_csv_prefix}_episodes.csv'

        self._samples_file = open(self._samples_csv_path, 'w', newline='')
        self._samples_writer = csv.DictWriter(self._samples_file, fieldnames=_SAMPLE_CSV_FIELDS)
        self._samples_writer.writeheader()
        self._samples_file.flush()

        self._episodes_file = open(self._episodes_csv_path, 'w', newline='')
        self._episodes_writer = csv.DictWriter(self._episodes_file, fieldnames=_EPISODE_CSV_FIELDS)
        self._episodes_writer.writeheader()
        self._episodes_file.flush()

    def _setup_interfaces(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            SparseVoxelGrid, self.voxel_grid_topic, self.on_voxel_grid, 10)
        self.create_subscription(
            Pose, self.goal_topic, self.on_goal, 10)

        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Path, self.path_topic, self.on_path, path_qos)

        # Matches grasp.py's state_pub QoS (TRANSIENT_LOCAL) so we pick up the
        # current state immediately on startup instead of waiting for the next transition.
        state_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, self.state_topic, self.on_state, state_qos)

        self.q_goal_pub = self.create_publisher(Float32, '/workspace_reachability/q_goal', 10)
        self.q_path_mean_pub = self.create_publisher(
            Float32, '/workspace_reachability/q_path_mean', 10)
        self.episode_q_goal_pub = self.create_publisher(
            Float32, '/workspace_reachability/episode_q_goal_mean', 10)
        self.episode_q_path_pub = self.create_publisher(
            Float32, '/workspace_reachability/episode_q_path_mean', 10)

    # ── Callbacks ────────────────────────────────────────────────────────────

    def on_voxel_grid(self, msg: SparseVoxelGrid):
        voxel_map = self.engine.preprocess(
            msg.occupied_indices, msg.size_x, msg.size_y, msg.size_z)
        self._rm = self.engine.predict(voxel_map)
        self._vg_info = {
            'origin': msg.origin,
            'resolution': float(msg.resolution),
            'size_x': msg.size_x, 'size_y': msg.size_y, 'size_z': msg.size_z,
        }
        self._vg_frame = msg.header.frame_id
        self._rm_stamp = self.get_clock().now()

    def on_goal(self, msg: Pose):
        # mpc_goal carries no header/frame; assume planning_frame (see grasp.py, where
        # this Pose is always expressed in the same frame passed to the trajectory action).
        self._goal = (msg.position.x, msg.position.y, msg.position.z)

    def on_state(self, msg: String):
        if msg.data == self._latest_state:
            return
        self._latest_state = msg.data
        if self._episode is not None:
            self._episode.add_state_event(time.time(), msg.data)

    def on_path(self, msg: Path):
        t_start = time.time()

        if self._rm is None:
            self.get_logger().warn(
                'No reachability map yet, skipping trajectory evaluation',
                throttle_duration_sec=5.0)
            return
        if self._goal is None:
            self.get_logger().warn(
                'No mpc_goal received yet, skipping trajectory evaluation',
                throttle_duration_sec=5.0)
            return
        if not msg.poses:
            return

        now = time.time()
        self._maybe_start_new_episode(now)
        self._last_path_time = now

        centers = self._extract_path_centers(msg)
        if not centers:
            return

        goal_eval = self._get_goal_evaluation(self._goal)
        q_goal = goal_eval.compute_quality_score(self._rm, **self._vg_info)

        q_path_values = WorkspaceEvaluation.compute_quality_scores_batched(
            self._rm, centers, radius=self.workspace_radius, device=self.device,
            **self._vg_info)
        q_path_mean = float(np.mean(q_path_values))
        q_path_min = float(np.min(q_path_values))
        q_path_max = float(np.max(q_path_values))

        compute_ms = (time.time() - t_start) * 1000.0
        rm_age_ms = (self.get_clock().now() - self._rm_stamp).nanoseconds / 1e6

        self._publish_and_log_sample(
            msg, centers, q_goal, q_path_mean, q_path_min, q_path_max,
            rm_age_ms, compute_ms)

        self._episode.add_sample(now, q_goal, q_path_mean, q_path_min, q_path_max, self._goal)
        self._seq += 1

    def _check_episode_timeout(self):
        if self._episode is None or self._last_path_time is None:
            return
        if time.time() - self._last_path_time > self.episode_timeout:
            self._flush_episode(time.time())

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _maybe_start_new_episode(self, now):
        start_new = self._episode is None
        if self._episode is not None and self._last_path_time is not None:
            if now - self._last_path_time > self.episode_timeout:
                self._flush_episode(now)
                start_new = True
        if not start_new and self.episode_goal_jump > 0.0 and self._episode.last_goal is not None:
            prev = np.array(self._episode.last_goal)
            cur = np.array(self._goal)
            if float(np.linalg.norm(cur - prev)) > self.episode_goal_jump:
                self._flush_episode(now)
                start_new = True

        if start_new:
            self._episode = _EpisodeAccumulator(self._next_episode_id, now)
            self._next_episode_id += 1

    def _flush_episode(self, t_end):
        if self._episode is None or self._episode.n_samples == 0:
            self._episode = None
            return

        summary = self._episode.summarize(t_end)
        self._episodes_writer.writerow(summary)
        self._episodes_file.flush()

        if self.generate_episode_plot:
            self._plot_episode(self._episode, summary)

        msg_goal = Float32()
        msg_goal.data = summary['q_goal_mean']
        self.episode_q_goal_pub.publish(msg_goal)

        msg_path = Float32()
        msg_path.data = summary['q_path_mean']
        self.episode_q_path_pub.publish(msg_path)

        self.get_logger().info(
            f"Episode {summary['episode_id']} done: n={summary['n_samples']} "
            f"({summary['rate_hz']:.1f} Hz over {summary['duration_s']:.1f}s) | "
            f"Q_goal_mean={summary['q_goal_mean']:.4f} "
            f"Q_path_mean={summary['q_path_mean']:.4f} "
            f"[{summary['q_path_min']:.4f}, {summary['q_path_max']:.4f}]"
        )

        self._episode = None
        self._last_path_time = None

    def _plot_episode(self, episode, summary):
        """Save a PNG of Q_goal / Q_path over the episode next to the CSV outputs."""
        path = f'{self.output_csv_prefix}_episode_{episode.episode_id:03d}.png'

        # Reserve a band at the top of the axes (in axes-fraction, independent of the
        # data range) for the rotated state-name labels, so they can never collide
        # with the title above the axes.
        label_band = 0.28 if episode.state_events else 0.0
        figsize = (8, 5.2) if episode.state_events else (8, 4.5)

        fig, ax = plt.subplots(figsize=figsize)
        t = episode.rel_times

        ax.fill_between(
            t, episode.q_path_min_series, episode.q_path_max_series,
            color='#e15759', alpha=0.15, label='Q_path range [min, max]')
        ax.plot(t, episode.q_path_means, marker='.', color='#e15759', linewidth=1.2,
                label='Q_path mean')
        ax.plot(t, episode.q_goal_values, marker='.', color='#4e79a7', linewidth=1.2,
                label='Q_goal')

        goal_mean_label = f"Q_goal episode mean ({summary['q_goal_mean']:.3f})"
        path_mean_label = f"Q_path episode mean ({summary['q_path_mean']:.3f})"
        ax.axhline(summary['q_goal_mean'], color='#4e79a7', linestyle='--',
                   linewidth=1, alpha=0.6, label=goal_mean_label)
        ax.axhline(summary['q_path_mean'], color='#e15759', linestyle='--',
                   linewidth=1, alpha=0.6, label=path_mean_label)

        data_top = 1.0 - label_band
        for rel_t, state_name in episode.state_events:
            ax.axvline(rel_t, ymax=data_top, color='#666666', linestyle=':',
                       linewidth=1, alpha=0.8)
            ax.text(rel_t, data_top + 0.01, f' {state_name}', transform=ax.get_xaxis_transform(),
                    rotation=90, va='bottom', ha='left', fontsize=7, color='#666666')

        ax.set_xlabel('Time since episode start (s)')
        ax.set_ylabel('Mean reachability Q')
        ax.set_title(
            f'Episode {episode.episode_id} — workspace reachability '
            f"(n={summary['n_samples']} samples, {summary['duration_s']:.1f}s)")
        ax.legend(fontsize=7, loc='lower right')
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)

        self.get_logger().info(f'Episode {episode.episode_id} plot saved to {path}')

    def _get_goal_evaluation(self, goal):
        """Reuse the cached WorkspaceEvaluation while the goal hasn't moved.

        WorkspaceEvaluation caches its sphere mask internally keyed on the voxel
        grid parameters, but rebuilding the object also invalidates that cache, so
        we additionally avoid recreating it every message when the goal is static.
        """
        if self._goal_eval is None or self._goal_eval_center != goal:
            self._goal_eval = WorkspaceEvaluation(
                centers_xyz=goal, radius=self.workspace_radius, device=self.device)
            self._goal_eval_center = goal
        return self._goal_eval

    def _extract_path_centers(self, msg: Path):
        """Subsample path poses (always keeping first/last) and transform to the RM frame."""
        n = len(msg.poses)
        if self.path_max_samples > 0 and n > self.path_max_samples:
            idx = np.linspace(0, n - 1, self.path_max_samples).round().astype(int)
            idx = sorted({int(i) for i in idx})
        else:
            idx = range(n)

        path_frame = msg.header.frame_id or self.planning_frame
        need_transform = path_frame != self._vg_frame

        tf = None
        if need_transform:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self._vg_frame, path_frame, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException) as ex:
                self.get_logger().warn(
                    f'TF {path_frame} -> {self._vg_frame} unavailable: {ex}',
                    throttle_duration_sec=5.0)
                return []

        centers = []
        for i in idx:
            pose_stamped = msg.poses[i]
            if need_transform:
                ps = PoseStamped()
                ps.header.frame_id = path_frame
                ps.pose = pose_stamped.pose
                transformed = do_transform_pose_stamped(ps, tf)
                p = transformed.pose.position
            else:
                p = pose_stamped.pose.position
            centers.append((p.x, p.y, p.z))
        return centers

    def _publish_and_log_sample(
            self, msg, centers, q_goal, q_path_mean, q_path_min, q_path_max,
            rm_age_ms, compute_ms):
        m_goal = Float32()
        m_goal.data = q_goal
        self.q_goal_pub.publish(m_goal)

        m_path = Float32()
        m_path.data = q_path_mean
        self.q_path_mean_pub.publish(m_path)

        gx, gy, gz = self._goal
        self._samples_writer.writerow({
            'timestamp': time.time(),
            'episode_id': self._episode.episode_id,
            'seq': self._seq,
            'goal_x': gx, 'goal_y': gy, 'goal_z': gz,
            'path_frame': msg.header.frame_id,
            'n_poses': len(msg.poses),
            'n_eval': len(centers),
            'q_goal': q_goal,
            'q_path_mean': q_path_mean,
            'q_path_min': q_path_min,
            'q_path_max': q_path_max,
            'rm_age_ms': rm_age_ms,
            'compute_ms': compute_ms,
        })
        self._samples_file.flush()

        if self.log_each_sample:
            self.get_logger().info(
                f'[ep {self._episode.episode_id}] Q_goal={q_goal:.4f} '
                f'Q_path_mean={q_path_mean:.4f} [{q_path_min:.4f}, {q_path_max:.4f}] '
                f'n={len(centers)}/{len(msg.poses)} rm_age={rm_age_ms:.0f}ms '
                f'compute={compute_ms:.1f}ms'
            )

    def destroy_node(self):
        if self._episode is not None:
            self._flush_episode(time.time())
        if hasattr(self, '_samples_file'):
            self._samples_file.close()
        if hasattr(self, '_episodes_file'):
            self._episodes_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceReachabilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
