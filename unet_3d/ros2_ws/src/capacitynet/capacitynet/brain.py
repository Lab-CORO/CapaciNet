#!/usr/bin/env python3
"""Minimal debug node for workspace definition + reachability quality.

Stripped-down sibling of brain_orchestrator: no state machine, no gripper,
no trajectory, no gradient control, no /cmd_vel. It only:

  1. Tracks a fiducial marker as the workspace center.
  2. Computes the reachability quality score Q of the spherical workspace at
     that single position (no 3x3 grid), from the live voxel grid.
  3. Visualizes the workspace as a SPHERE marker in RViz.

Q is logged to the console only.
"""

import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from curobo_msgs.msg import SparseVoxelGrid
from ros2_markertracker_interfaces.msg import FiducialMarkerArray

from .reachability_engine import ReachabilityEngine
from .workspace_evaluation import WorkspaceEvaluation


class BrainDebug(Node):
    """Workspace visualization + single-position reachability quality."""

    def __init__(self):
        super().__init__('brain')

        self._load_parameters()

        # Latest fiducial marker pose (in its own camera frame), or None.
        self._marker_pose = None

        self._setup_engine()
        self._setup_interfaces()

        self.get_logger().info("Brain debug node initialized")
        self.get_logger().info(f"  - Marker topic: {self.marker_topic}")
        self.get_logger().info(f"  - Voxel grid topic: {self.voxel_grid_topic}")
        self.get_logger().info(f"  - Target marker id: {self.target_marker_id} (-1 = first available)")
        self.get_logger().info(f"  - Workspace radius: {self.workspace_radius} m")

    def _load_parameters(self):
        self.declare_parameter('marker_topic', '/fiducial_markers')
        self.declare_parameter('voxel_grid_topic', '/curobo_trajectory_planner/voxel_grid_sparse')
        # Marker id used as workspace center; -1 = use the first marker in the array.
        self.declare_parameter('target_marker_id', -1)
        self.declare_parameter('workspace_radius', 0.30)
        self.declare_parameter('model_config_path', '/home/ros2_ws/src/capacitynet/config/test_reach.yaml')

        self.marker_topic = self.get_parameter('marker_topic').value
        self.voxel_grid_topic = self.get_parameter('voxel_grid_topic').value
        self.target_marker_id = self.get_parameter('target_marker_id').value
        self.workspace_radius = self.get_parameter('workspace_radius').value
        self.model_config_path = self.get_parameter('model_config_path').value

    def _setup_engine(self):
        import torch
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Loading reachability model on {self.device}...")
        self.engine = ReachabilityEngine(self.model_config_path)
        self.get_logger().info("Reachability model loaded")

    def _setup_interfaces(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_sub = self.create_subscription(
            FiducialMarkerArray, self.marker_topic, self.on_markers, 10
        )
        self.voxel_grid_sub = self.create_subscription(
            SparseVoxelGrid, self.voxel_grid_topic, self.on_voxel_grid, 10
        )
        self.workspace_marker_pub = self.create_publisher(Marker, '/workspace_marker', 10)

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

    def on_voxel_grid(self, msg: SparseVoxelGrid):
        """Compute and log the workspace quality at the tracked marker position."""
        if self._marker_pose is None:
            self.get_logger().warn("No marker received yet, skipping quality computation",
                                   throttle_duration_sec=5.0)
            return

        target_frame = msg.header.frame_id
        center = self._transform_marker_to(target_frame)
        if center is None:
            return

        # Build reachability map for the current scene (single position).
        voxel_map = self.engine.preprocess(
            msg.occupied_indices, msg.size_x, msg.size_y, msg.size_z
        )
        rm = self.engine.predict(voxel_map)

        workspace_eval = WorkspaceEvaluation(
            centers_xyz=center, radius=self.workspace_radius, device=self.device
        )
        quality = workspace_eval.compute_quality_score(
            rm, msg.origin, msg.resolution, msg.size_x, msg.size_y, msg.size_z
        )

        self.get_logger().info(
            f"Q={quality:.4f} | center=({center[0]:+.3f}, {center[1]:+.3f}, "
            f"{center[2]:+.3f}) [{target_frame}] r={self.workspace_radius}m"
        )

        self.publish_workspace_marker(center, target_frame)

        del rm, voxel_map

    # ── Helpers ──────────────────────────────────────────────────────────────

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
                f"TF {source_frame} -> {target_frame} unavailable: {ex}",
                throttle_duration_sec=5.0
            )
            return None

        transformed = do_transform_pose_stamped(self._marker_pose, tf)
        p = transformed.pose.position
        return (p.x, p.y, p.z)

    def publish_workspace_marker(self, center, frame_id):
        """Publish the spherical workspace as an RViz SPHERE marker."""
        diameter = 2.0 * self.workspace_radius

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'workspace'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3
        self.workspace_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = BrainDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
