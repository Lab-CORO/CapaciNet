#!/usr/bin/env python3
"""Publish a mock MPC path: the midpoint of the segment TCP <-> object.

Stands in for the real curobo MPC bridge when no planner is running. Every
tick it looks up two TF frames (the arm TCP and the object/target) in
`planning_frame`, takes the midpoint of the segment between them, and
publishes that single point as a one-pose `nav_msgs/Path` on `path_topic`.

No `mpc_goal` is published: WorkspaceRegionSource._mpc_region() falls back to
the path tail alone when no fresh mpc_goal exists (region_has_goal=False —
see workspace_region.py), which is exactly this mock's case. Publishing the
same point on both topics would make the goal sphere and the path sphere
concentric — the union collapses to whichever radius is larger, so the path
sphere is visually indistinguishable from the goal sphere in any debug dump.
Publishing only the path avoids that entirely.

on_path() takes `msg.poses[-path_tail_samples:]` — the last N poses of the
path. With a single-pose Path, that slice is just that one pose for any
`path_tail_samples >= 1`, so a real controller (which normally reads the
*last* pose of a multi-pose trajectory) works unmodified against this mock.

Orientation is left at identity: WorkspaceRegionSource only reads
`.position`.

Usage:
  ros2 run capacitynet mock_arm_trajectory --ros-args \
    -p tcp_frame:=dsr01/link_6 -p object_frame:=object_frame_name
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
import tf2_ros
from tf2_ros import Buffer, TransformListener


class MockArmTrajectory(Node):
    """Publishes the TCP-object segment midpoint as a mock MPC path."""

    def __init__(self):
        super().__init__('mock_arm_trajectory')

        self.declare_parameter('tcp_frame', 'dsr01/link_6')
        self.declare_parameter('object_frame', '')
        self.declare_parameter('planning_frame', 'dsr01/base_link')
        self.declare_parameter('path_topic', '/mpc_predicted_path')
        self.declare_parameter('publish_rate', 10.0)

        self.tcp_frame = self.get_parameter('tcp_frame').value
        self.object_frame = self.get_parameter('object_frame').value
        self.planning_frame = self.get_parameter('planning_frame').value
        self.path_topic = self.get_parameter('path_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        if not self.object_frame:
            raise ValueError(
                'object_frame is required (no safe default — set it to the TF '
                'frame of the grasp target)')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, self.path_topic, 1)

        period = 1.0 / max(publish_rate, 0.001)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'Mock arm trajectory: midpoint of {self.tcp_frame} <-> '
            f'{self.object_frame}, expressed in {self.planning_frame}')
        self.get_logger().info(
            f'  - Publishing {self.path_topic} (Path, 1 pose, no mpc_goal) '
            f'at {publish_rate:.1f} Hz')

    def _on_timer(self):
        tcp = self._lookup(self.tcp_frame)
        obj = self._lookup(self.object_frame)
        if tcp is None or obj is None:
            return

        midpoint = tuple((a + b) / 2.0 for a, b in zip(tcp, obj))
        stamp = self.get_clock().now().to_msg()

        path = Path()
        path.header.frame_id = self.planning_frame
        path.header.stamp = stamp
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = midpoint
        pose.pose.orientation.w = 1.0
        path.poses = [pose]
        self.path_pub.publish(path)

    def _lookup(self, frame):
        """Translation of `frame` in `planning_frame`, or None on TF failure."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.planning_frame, frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as ex:
            self.get_logger().warn(
                f'TF {frame} -> {self.planning_frame} unavailable: {ex}',
                throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)


def main(args=None):
    rclpy.init(args=args)
    node = MockArmTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
