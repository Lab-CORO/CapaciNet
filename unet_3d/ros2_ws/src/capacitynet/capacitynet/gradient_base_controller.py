#!/usr/bin/env python3
"""Move the mobile base toward better arm reachability.

Runs the reachability pipeline on live voxel grids and publishes a velocity
command that climbs the reachability gradient:

    SparseVoxelGrid -> ReachabilityEngine.preprocess
                    -> ObstacleMapTransformer (3x3 grid of shifted obstacle maps)
                    -> ReachabilityEngine.predict_batch  (9 reachability maps)
                    -> GradientBasedController.compute_control  (Q_i, grad Q, Twist)

This is the real-data counterpart of `gradient_controller_mock`, and replaces the
`enable_gradient_control` branch of `brain_orchestrator` (now off by default) so a
single node owns /cmd_vel.

What Q is scored over: the union of spheres around the MPC goal and the last few
poses of the MPC predicted path — the approach corridor the arm actually
traverses. The bare fiducial marker is *not* that target: grasp.py offsets the
tag by `approach_distance` (-0.2 m by default) along the marker Z to build the
pose it plans to, so scoring the tag alone optimises around a point the arm never
visits. The marker remains the fallback when no MPC goal is available.

Structure follows brain.py; the pipeline body follows brain_orchestrator, with
three things that path was missing:

  * TF. brain_orchestrator feeds raw camera-frame marker coordinates into a
    controller that scores against a voxel grid in the arm base frame. Here every
    center is transformed into the voxel grid's own frame (as brain.py does).
  * Fail-safe. A watchdog publishes an exact zero Twist when disabled, when data
    goes stale, when the grasper is busy, and on shutdown.
  * A speed cap tied to inference latency, so the base cannot outrun the region
    the gradient was sampled over.

Safety model: the node starts DISABLED and must be enabled explicitly, and it
disables itself whenever `grasp.py` reports a non-idle state.
"""

import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions

import tf2_ros
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped

from geometry_msgs.msg import Point, Pose, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
from curobo_msgs.msg import SparseVoxelGrid
from ros2_markertracker_interfaces.msg import FiducialMarkerArray

from .reachability_engine import ReachabilityEngine
from .obstacle_transformer import ObstacleMapTransformer
from .gradient_controller import GradientBasedController

ZERO_TWIST = Twist()


class GradientBaseController(Node):
    """Reachability-gradient velocity controller for the mobile base."""

    def __init__(self):
        super().__init__('gradient_base_controller')

        self._load_parameters()

        # Latest fiducial marker pose (in its own camera frame), or None.
        self._marker_pose = None
        self._marker_stamp = None

        # Latest MPC goal (planning_frame) and predicted-path tail (own frame).
        self._mpc_goal = None
        self._mpc_goal_stamp = None
        self._path_tail = None
        self._path_frame = None
        self._path_stamp = None
        # Which source produced the last region; logged when it changes, because
        # switching source changes what Q measures and so rescales the gradient.
        self._center_source = None
        self._n_region_markers = 0

        # Last computed command, republished by the watchdog timer until it goes stale.
        self._last_twist = Twist()
        self._last_cmd_time = None

        self._enabled = self.start_enabled
        self._converged = False
        # None = never heard from the grasper. Treated as permissive so the
        # controller still runs standalone when grasp.py is not up.
        self._grasper_state = None
        # EWMA of the measured cycle time; None until the first cycle completes.
        self._cycle_ewma = None

        self._setup_engine()
        self._setup_interfaces()

        self.get_logger().info('Gradient base controller initialized')
        self.get_logger().info(f'  - Voxel grid topic: {self.voxel_grid_topic}')
        self.get_logger().info(f'  - Marker topic: {self.marker_topic}')
        self.get_logger().info(
            f'  - Region: goal={self.goal_topic} path={self.path_topic} '
            f'(tail={self.path_tail_samples}), fallback=marker')
        self.get_logger().info(f'  - Command topic: {self.cmd_vel_topic}')
        self.get_logger().info(f'  - Base speed: {self.base_speed} m/s, delta: {self.grid_spacing} m')
        self.get_logger().info(f'  - Gradient method: {self.gradient_method}')
        self.get_logger().info(f'  - Backend: {"TensorRT" if self.engine.trt_model is not None else "PyTorch"}')
        if self.static_workspace_center:
            self.get_logger().info(
                f'  - Static workspace center: {self.static_workspace_center} '
                '(overrides MPC goal and marker)')
        self.get_logger().info(
            f'  - Enabled: {self._enabled} (toggle via {self.get_name()}/enable)')

    # ── Setup ────────────────────────────────────────────────────────────────

    def _load_parameters(self):
        self.declare_parameter('voxel_grid_topic', '/curobo_trajectory_planner/voxel_grid_sparse')
        self.declare_parameter('marker_topic', '/fiducial_markers')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('target_marker_id', -1)
        self.declare_parameter('model_config_path',
                               '/home/ros2_ws/src/capacitynet/config/test_reach.yaml')
        self.declare_parameter('fp16', False)

        self.declare_parameter('workspace_radius', 0.30)
        self.declare_parameter('grid_spacing', 0.10)

        # Motion. base_speed is the hardware-validated ceiling; the adaptive cap
        # below can only lower it, never raise it.
        self.declare_parameter('base_speed', 0.02)
        self.declare_parameter('step_fraction', 0.5)
        self.declare_parameter('min_speed', 0.002)
        self.declare_parameter('control_gain', 1.0)
        self.declare_parameter('gradient_taper_ref', 0.0)
        self.declare_parameter('gradient_method', 'least_squares')

        self.declare_parameter('use_static_obstacles', False)
        self.declare_parameter('static_obstacles_yaml',
                               '/home/ros2_ws/src/capacitynet/config/floor_world.yml')

        # Workspace region source. grasp.py republishes the offset grasp target on
        # mpc_goal at 10 Hz while approaching, and the planner publishes the
        # predicted path; together they describe the corridor to score.
        self.declare_parameter('goal_topic', '/curobo_trajectory_planner/mpc_goal')
        self.declare_parameter('path_topic', '/mpc_predicted_path')
        # How many poses from the END of the path join the goal in the union. The
        # tail is anchored near the goal by the tag geometry, so it stays stable
        # across replans, unlike the full trajectory. 0 -> goal sphere only.
        self.declare_parameter('path_tail_samples', 4)
        # mpc_goal is a bare Pose with no header; assume this frame (grasp.py
        # always publishes it in the frame it passes to the trajectory action).
        self.declare_parameter('planning_frame', 'dsr01/world')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('command_timeout', 1.0)
        self.declare_parameter('marker_timeout', 2.0)
        self.declare_parameter('mpc_timeout', 2.0)

        self.declare_parameter('start_enabled', False)
        self.declare_parameter('grasper_state_topic', '/object_grasper/state')
        self.declare_parameter('grasper_idle_state', 'idle')
        # Fallback workspace center, expressed in the voxel grid frame. Needed for
        # bag replay: no recorded bag contains /fiducial_markers. Kept as a string
        # ("x,y,z" or "[x, y, z]") so it passes identically through a launch
        # argument and a --ros-args -p override.
        self.declare_parameter('static_workspace_center', '')

        self.declare_parameter('log_timing', True)
        self.declare_parameter('log_quality_scores', False)
        self.declare_parameter('publish_debug_markers', True)

        g = self.get_parameter
        self.voxel_grid_topic = g('voxel_grid_topic').value
        self.marker_topic = g('marker_topic').value
        self.cmd_vel_topic = g('cmd_vel_topic').value
        self.target_marker_id = int(g('target_marker_id').value)
        self.model_config_path = g('model_config_path').value
        self.fp16 = bool(g('fp16').value)

        self.workspace_radius = float(g('workspace_radius').value)
        self.grid_spacing = float(g('grid_spacing').value)

        self.base_speed = float(g('base_speed').value)
        self.step_fraction = float(g('step_fraction').value)
        self.min_speed = float(g('min_speed').value)
        self.control_gain = float(g('control_gain').value)
        self.gradient_taper_ref = float(g('gradient_taper_ref').value)
        self.gradient_method = g('gradient_method').value

        self.use_static_obstacles = bool(g('use_static_obstacles').value)
        self.static_obstacles_yaml = g('static_obstacles_yaml').value

        self.goal_topic = g('goal_topic').value
        self.path_topic = g('path_topic').value
        self.path_tail_samples = int(g('path_tail_samples').value)
        self.planning_frame = g('planning_frame').value

        self.publish_rate = float(g('publish_rate').value)
        self.command_timeout = float(g('command_timeout').value)
        self.marker_timeout = float(g('marker_timeout').value)
        self.mpc_timeout = float(g('mpc_timeout').value)

        self.start_enabled = bool(g('start_enabled').value)
        self.grasper_state_topic = g('grasper_state_topic').value
        self.grasper_idle_state = g('grasper_idle_state').value
        self.static_workspace_center = self._parse_center(g('static_workspace_center').value)

        self.log_timing = bool(g('log_timing').value)
        self.log_quality_scores = bool(g('log_quality_scores').value)
        self.publish_debug_markers = bool(g('publish_debug_markers').value)

    @staticmethod
    def _parse_center(raw):
        """Parse a 'x,y,z' / '[x, y, z]' workspace center. Empty string -> None."""
        if not raw or not str(raw).strip():
            return None
        parts = [p for p in str(raw).replace('[', ' ').replace(']', ' ')
                 .replace(',', ' ').split() if p]
        if len(parts) != 3:
            raise ValueError(
                f'static_workspace_center must have 3 components, got {raw!r}')
        return tuple(float(p) for p in parts)

    def _setup_engine(self):
        import torch
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Loading reachability model on {self.device}...')
        self.engine = ReachabilityEngine(self.model_config_path, fp16=self.fp16)

        # Resolution is a placeholder; it is overwritten per message from the
        # live grid via update_resolution().
        self.obstacle_transformer = ObstacleMapTransformer(
            resolution=0.02,
            device=self.device,
            static_obstacles_yaml=self.static_obstacles_yaml if self.use_static_obstacles else None,
        )
        self.gradient_ctrl = GradientBasedController(
            workspace_radius=self.workspace_radius,
            grid_spacing=self.grid_spacing,
            gain=self.control_gain,
            max_linear_vel=self.base_speed,
            device=self.device,
            gradient_method=self.gradient_method,
        )
        self.get_logger().info('Reachability model loaded')

    def _setup_interfaces(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # All CUDA work is confined to this group so exactly one inference runs at
        # a time. Everything else lives in the control group, which must keep
        # publishing while a ~440 ms inference is in flight — on a single-threaded
        # executor /cmd_vel would collapse to the inference rate (~2 Hz) and most
        # base drivers time out below ~4 Hz.
        self.cuda_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()

        self.voxel_grid_sub = self.create_subscription(
            SparseVoxelGrid, self.voxel_grid_topic, self.on_voxel_grid, 1,
            callback_group=self.cuda_group)

        self.marker_sub = self.create_subscription(
            FiducialMarkerArray, self.marker_topic, self.on_markers, 10,
            callback_group=self.control_group)

        # Only the newest goal/path matter — an older one describes a corridor the
        # arm has already left, so keep depth 1 rather than queueing.
        if self.goal_topic:
            self.goal_sub = self.create_subscription(
                Pose, self.goal_topic, self.on_mpc_goal, 1,
                callback_group=self.control_group)
        if self.path_topic and self.path_tail_samples > 0:
            path_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.path_sub = self.create_subscription(
                Path, self.path_topic, self.on_path, path_qos,
                callback_group=self.control_group)

        if self.grasper_state_topic:
            # Transient-local to match grasp.py's publisher, otherwise the latched
            # current state is missed and only the next transition is seen.
            state_qos = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.grasper_state_sub = self.create_subscription(
                String, self.grasper_state_topic, self.on_grasper_state, state_qos,
                callback_group=self.control_group)

        self.enable_sub = self.create_subscription(
            Bool, '~/enable', self.on_enable_msg, 10,
            callback_group=self.control_group)
        self.enable_srv = self.create_service(
            SetBool, '~/enable', self.on_enable_srv,
            callback_group=self.control_group)

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        if self.publish_debug_markers:
            self.grid_marker_pub = self.create_publisher(MarkerArray, '~/grid_markers', 10)

        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_command,
            callback_group=self.control_group)

    # ── Enable / interlock ───────────────────────────────────────────────────

    def _set_enabled(self, value, source):
        value = bool(value)
        if value != self._enabled:
            self.get_logger().info(f'{"Enabled" if value else "Disabled"} via {source}')
        self._enabled = value
        self._converged = False
        if not value:
            self._stop()

    def on_enable_msg(self, msg: Bool):
        self._set_enabled(msg.data, 'topic')

    def on_enable_srv(self, request, response):
        self._set_enabled(request.data, 'service')
        response.success = True
        response.message = 'enabled' if self._enabled else 'disabled'
        return response

    def on_grasper_state(self, msg: String):
        previous = self._grasper_state
        self._grasper_state = msg.data
        if msg.data != self.grasper_idle_state and previous != msg.data:
            self.get_logger().info(f"Grasper busy ('{msg.data}') — holding base still")
            self._stop()

    def _grasper_allows_motion(self):
        # Unknown state (grasper not running) is permissive by design.
        return self._grasper_state is None or self._grasper_state == self.grasper_idle_state

    # ── Callbacks ────────────────────────────────────────────────────────────

    def on_markers(self, msg: FiducialMarkerArray):
        """Store the pose of the tracked marker (in its own camera frame)."""
        if not msg.marker:
            return

        selected = None
        if self.target_marker_id < 0:
            selected = msg.marker[0]
        else:
            for m in msg.marker:
                if m.id == self.target_marker_id:
                    selected = m
                    break

        if selected is None:
            return

        pose = PoseStamped()
        pose.header = selected.pose_cov_stamped.header
        pose.pose = selected.pose_cov_stamped.pose.pose
        self._marker_pose = pose
        self._marker_stamp = self.get_clock().now()

    def on_mpc_goal(self, msg: Pose):
        """Store the grasp target grasp.py is driving the arm to."""
        self._mpc_goal = (msg.position.x, msg.position.y, msg.position.z)
        self._mpc_goal_stamp = self.get_clock().now()

    def on_path(self, msg: Path):
        """Store the tail of the predicted path, in the path's own frame."""
        if not msg.poses:
            return
        tail = msg.poses[-self.path_tail_samples:]
        self._path_tail = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                           for p in tail]
        self._path_frame = msg.header.frame_id or self.planning_frame
        self._path_stamp = self.get_clock().now()

    def on_voxel_grid(self, msg: SparseVoxelGrid):
        """Run one control cycle: 9 reachability maps -> grad Q -> velocity."""
        if not self._enabled or self._converged:
            return
        if not self._grasper_allows_motion():
            return

        centers = self._resolve_workspace_centers(msg.header.frame_id)
        if centers is None:
            return

        sx, sy, sz = msg.size_x, msg.size_y, msg.size_z
        vg_info = {
            'origin': msg.origin,
            'resolution': float(msg.resolution),
            'size_x': sx, 'size_y': sy, 'size_z': sz,
        }
        if not self._workspace_fits(centers, vg_info):
            return

        t_start = time.time()

        voxel_map = self.engine.preprocess(msg.occupied_indices, sx, sy, sz)
        self.obstacle_transformer.update_resolution(vg_info['resolution'])
        transformed = self.obstacle_transformer.generate_grid_transforms(
            voxel_map, grid_spacing=self.gradient_ctrl.delta, origin=vg_info['origin'])

        # Cap speed so the base cannot travel more than step_fraction * delta
        # between two gradient updates. Uses the previous cycle's duration; the
        # first cycle runs uncapped at base_speed.
        v_cap = self._velocity_cap()
        self.gradient_ctrl.max_linear_vel = v_cap

        rm_9 = self.engine.predict_batch(transformed)

        self.gradient_ctrl.update_workspace_region(centers)
        twist, debug = self.gradient_ctrl.compute_control(rm_9, vg_info)
        twist = self._shape_command(twist, debug)

        self._last_twist = twist
        self._last_cmd_time = self.get_clock().now()

        elapsed = time.time() - t_start
        self._cycle_ewma = elapsed if self._cycle_ewma is None else (
            0.3 * elapsed + 0.7 * self._cycle_ewma)

        self._log_cycle(debug, elapsed, v_cap)
        if self.publish_debug_markers:
            self._publish_grid_markers(centers, debug, msg.header.frame_id)

        del rm_9, transformed, voxel_map

    def publish_command(self):
        """Watchdog: republish the last command, or zero when it cannot be trusted."""
        if not self._enabled or self._converged or not self._grasper_allows_motion():
            self.cmd_vel_pub.publish(ZERO_TWIST)
            return

        if self._last_cmd_time is None:
            self.cmd_vel_pub.publish(ZERO_TWIST)
            return

        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self.command_timeout:
            self.get_logger().warn(
                f'No reachability update for {age:.1f}s (> {self.command_timeout}s), stopping base',
                throttle_duration_sec=5.0)
            self.cmd_vel_pub.publish(ZERO_TWIST)
            return

        self.cmd_vel_pub.publish(self._last_twist)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _age_s(self, stamp):
        """Seconds since `stamp`, or +inf if it was never set."""
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _note_source(self, source):
        """Log region-source changes: they rescale Q, and so |grad Q|."""
        if source != self._center_source:
            if self._center_source is not None:
                self.get_logger().info(
                    f'Workspace region source: {self._center_source} -> {source} '
                    '(Q now covers a different region, |grad Q| rescales)')
            self._center_source = source

    def _resolve_workspace_centers(self, target_frame):
        """Workspace region in `target_frame` as a list of centers, or None.

        Priority: the explicit static override, then the MPC goal together with
        the tail of the predicted path, then the fiducial marker. Q is the mean
        over the *union* of the spheres, so several centers describe the approach
        corridor rather than a single point.
        """
        if self.static_workspace_center is not None:
            self._note_source('static')
            return [self.static_workspace_center]

        centers = self._mpc_region(target_frame)
        if centers:
            self._note_source(f'mpc_goal+{len(centers) - 1}path')
            return centers

        centers = self._marker_region(target_frame)
        if centers:
            self._note_source('marker')
            return centers
        return None

    def _mpc_region(self, target_frame):
        """Goal sphere plus the path tail, in target_frame. [] when unavailable."""
        if self._mpc_goal is None or self._age_s(self._mpc_goal_stamp) > self.mpc_timeout:
            return []

        centers = self._transform_positions(
            [self._mpc_goal], self.planning_frame, target_frame)
        if not centers:
            return []

        if self._path_tail and self._age_s(self._path_stamp) <= self.mpc_timeout:
            centers.extend(self._transform_positions(
                self._path_tail, self._path_frame, target_frame))
        return centers

    def _marker_region(self, target_frame):
        """Single sphere on the fiducial marker. [] when unavailable."""
        if self._marker_pose is None:
            self.get_logger().warn(
                'No workspace target yet (no MPC goal, no marker), not commanding base',
                throttle_duration_sec=5.0)
            return []

        age = self._age_s(self._marker_stamp)
        if age > self.marker_timeout:
            self.get_logger().warn(f'Marker stale ({age:.1f}s), not commanding base',
                                   throttle_duration_sec=5.0)
            self._stop()
            return []

        center = self._transform_marker_to(target_frame)
        return [center] if center is not None else []

    def _transform_positions(self, positions, source_frame, target_frame):
        """Transform (x, y, z) tuples between frames with one TF lookup. [] on failure."""
        if source_frame == target_frame:
            return list(positions)

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as ex:
            self.get_logger().warn(
                f'TF {source_frame} -> {target_frame} unavailable: {ex}',
                throttle_duration_sec=5.0)
            return []

        transformed = []
        for x, y, z in positions:
            ps = PoseStamped()
            ps.header.frame_id = source_frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            p = do_transform_pose_stamped(ps, tf).pose.position
            transformed.append((p.x, p.y, p.z))
        return transformed

    def _transform_marker_to(self, target_frame):
        """Transform the stored marker pose into target_frame. Returns (x,y,z) or None."""
        source_frame = self._marker_pose.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as ex:
            self.get_logger().warn(
                f'TF {source_frame} -> {target_frame} unavailable: {ex}',
                throttle_duration_sec=5.0
            )
            return None

        transformed = do_transform_pose_stamped(self._marker_pose, tf)
        p = transformed.pose.position
        return (p.x, p.y, p.z)

    def _workspace_fits(self, centers, vg_info):
        """True if every region sphere stays inside the grid at all 9 offsets.

        VoxelMask.compute_mean returns 0.0 for an empty mask, so a sphere that
        leaves the grid scores as *zero reachability* rather than *unknown* and
        would fabricate a large gradient pointing away from the boundary. The
        scored centers are `center - offset`, so they range over +/- delta in x
        and y. Every center is checked: a partially observed corridor would bias
        Q just as badly as a partially observed goal.
        """
        origin = vg_info['origin']
        res = vg_info['resolution']
        lo = (origin.x, origin.y, origin.z)
        hi = (origin.x + vg_info['size_x'] * res,
              origin.y + vg_info['size_y'] * res,
              origin.z + vg_info['size_z'] * res)
        # Z is never offset by the 3x3 grid, only X and Y.
        margins = (self.workspace_radius + self.gradient_ctrl.delta,
                   self.workspace_radius + self.gradient_ctrl.delta,
                   self.workspace_radius)

        for idx, center in enumerate(centers):
            for axis, (c, m) in enumerate(zip(center, margins)):
                if c - m < lo[axis] or c + m > hi[axis]:
                    which = 'goal' if idx == 0 else f'path point {idx}'
                    self.get_logger().warn(
                        f'Workspace sphere ({which}) leaves the voxel grid on axis '
                        f'{"xyz"[axis]} (center={c:+.3f}, margin={m:.3f}, '
                        f'bounds=[{lo[axis]:+.3f}, {hi[axis]:+.3f}]) — skipping cycle',
                        throttle_duration_sec=5.0)
                    return False
        return True

    def _velocity_cap(self):
        """Speed ceiling keeping travel per cycle within step_fraction * delta."""
        if self._cycle_ewma is None or self._cycle_ewma <= 0.0:
            return self.base_speed
        cap = self.step_fraction * self.gradient_ctrl.delta / self._cycle_ewma
        return min(self.base_speed, cap)

    def _shape_command(self, twist, debug):
        """Apply the optional taper and the min_speed deadband."""
        vx, vy = twist.linear.x, twist.linear.y

        # Optional quadratic taper: sharpens deceleration into the optimum. Only
        # active below g_ref, where saturation no longer binds, so it does not
        # slow the approach from far away.
        if self.gradient_taper_ref > 0.0:
            scale = min(1.0, debug['gradient_magnitude'] / self.gradient_taper_ref)
            vx *= scale
            vy *= scale

        speed = math.hypot(vx, vy)
        if speed < self.min_speed:
            self._converged = True
            self.get_logger().info(
                f'Converged: |grad Q|={debug["gradient_magnitude"]:.5f} gives '
                f'{speed:.4f} m/s < min_speed={self.min_speed} m/s. '
                f'Q_center={debug["score_center"]:.4f}. Re-enable to resume.')
            return Twist()

        return self.gradient_ctrl.create_twist_message(vx, vy)

    def _log_cycle(self, debug, elapsed, v_cap):
        if self.log_quality_scores:
            s = debug['scores']
            self.get_logger().info(
                'Q scores:\n'
                f'  {s[0]:.4f}  {s[1]:.4f}  {s[2]:.4f}\n'
                f'  {s[3]:.4f}  {s[4]:.4f}  {s[5]:.4f}\n'
                f'  {s[6]:.4f}  {s[7]:.4f}  {s[8]:.4f}')

        if debug['mask_shift_exact'] is False:
            # delta is not a whole number of voxels, so the 9 region masks cannot
            # be integer-shifted copies and each is rebuilt from its own centers.
            self.get_logger().warn(
                f'grid_spacing={self.gradient_ctrl.delta} m is not a whole number of '
                f'voxels — rebuilding {debug["region_size"]} spheres per grid position '
                'instead of shifting one mask',
                throttle_duration_sec=30.0)

        if self.log_timing:
            gx, gy = debug['gradient']
            vx, vy = self._last_twist.linear.x, self._last_twist.linear.y
            self.get_logger().info(
                f'grad Q=({gx:+.4f},{gy:+.4f}) |{debug["gradient_magnitude"]:.4f}| '
                f'v=({vx:+.4f},{vy:+.4f}) m/s '
                f'Q_c={debug["score_center"]:.4f} '
                f'region={self._center_source}({debug["region_size"]}) '
                f'cycle={elapsed * 1000:.0f}ms cap={v_cap:.3f}m/s')

    def _publish_grid_markers(self, centers, debug, frame_id):
        """Visualize the 9 candidate scores, laid out in base-motion space.

        Sphere i sits at `center + offset_i`, so the brightest sphere marks the
        direction the *base* should move. (The scored workspace center is
        `center - offset_i`; the sign is flipped here so the display reads as a
        motion suggestion rather than a workspace displacement.)

        The region actually scored — goal plus path tail — is drawn separately at
        the workspace radius, so what Q covers is visible rather than inferred.
        """
        center = centers[0]
        scores = debug['scores']
        lo, hi = float(scores.min()), float(scores.max())
        span = (hi - lo) or 1.0
        best = int(scores.argmax())
        now = self.get_clock().now().to_msg()

        array = MarkerArray()
        for i in range(9):
            ox, oy = self.gradient_ctrl._get_grid_offset(i)
            t = (float(scores[i]) - lo) / span

            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = now
            m.ns = 'reachability_grid'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = center[0] + ox
            m.pose.position.y = center[1] + oy
            m.pose.position.z = center[2]
            m.pose.orientation.w = 1.0
            d = 0.05 if i != best else 0.08
            m.scale.x = m.scale.y = m.scale.z = d
            m.color.r = t
            m.color.g = 0.2
            m.color.b = 1.0 - t
            m.color.a = 0.9 if i == best else 0.6
            array.markers.append(m)

        gx, gy = debug['gradient']
        arrow = Marker()
        arrow.header.frame_id = frame_id
        arrow.header.stamp = now
        arrow.ns = 'reachability_grid'
        arrow.id = 9
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        norm = math.hypot(gx, gy) or 1.0
        arrow.points = [
            Point(x=center[0], y=center[1], z=center[2]),
            Point(x=center[0] + 0.15 * gx / norm,
                  y=center[1] + 0.15 * gy / norm,
                  z=center[2]),
        ]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.01, 0.02, 0.02
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 1.0, 0.0, 0.9
        array.markers.append(arrow)

        # The scored region itself, at true radius. Ids 10.. so they never collide
        # with the 3x3 grid (0-8) or the gradient arrow (9).
        for i, c in enumerate(centers):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = now
            m.ns = 'workspace_region'
            m.id = 10 + i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = c
            m.pose.orientation.w = 1.0
            d = 2.0 * self.workspace_radius
            m.scale.x = m.scale.y = m.scale.z = d
            # Goal opaque-ish, path tail fainter — the union is what Q averages over.
            m.color.r, m.color.g, m.color.b = (0.1, 0.9, 0.4) if i == 0 else (0.9, 0.6, 0.1)
            m.color.a = 0.12 if i == 0 else 0.07
            array.markers.append(m)

        # Drop region spheres left over from a larger previous region.
        for i in range(len(centers), self._n_region_markers):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = now
            m.ns = 'workspace_region'
            m.id = 10 + i
            m.action = Marker.DELETE
            array.markers.append(m)
        self._n_region_markers = len(centers)

        self.grid_marker_pub.publish(array)

    def _stop(self):
        """Command an immediate stop and forget the cached velocity."""
        self._last_twist = Twist()
        self._last_cmd_time = None
        self.cmd_vel_pub.publish(ZERO_TWIST)

    def stop_base(self):
        """Best-effort stop, safe to call while the context is being torn down."""
        try:
            self._enabled = False
            self._stop()
        except Exception as ex:  # noqa: BLE001 - shutdown must never raise
            self.get_logger().warn(f'Could not publish stop command: {ex}')


def main(args=None):
    # rclpy's own SIGINT handler shuts the context down before user code runs,
    # which would make the final stop command unpublishable and leave the base
    # holding its last velocity. Handle the signals here instead so the stop
    # goes out while the context is still valid.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GradientBaseController()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    def _handle_signal(_signum, _frame):
        # Publish the stop first, while the context is still valid, then shut the
        # context down — that wakes the executor's wait set and ends spin().
        node.stop_base()
        if rclpy.ok():
            rclpy.shutdown()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _handle_signal)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().info('Shutting down, base stopped')
        node.stop_base()
        # Bounded so a hung callback cannot block exit; one cycle is ~0.5 s.
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
