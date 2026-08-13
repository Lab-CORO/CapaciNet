#!/usr/bin/env python3
"""Publish a fictive fiducial marker, dead-reckoned against /cmd_vel.

Stands in for ros2_markertracker when no real ArTag is in view. The marker is
anchored at a fixed pose *in the frame the base occupied when this node
started* (not in `base_frame` itself, which moves with the robot). Every
tick, the node dead-reckons the base's pose in that anchor frame by
integrating the cached /cmd_vel twist, then republishes the marker's pose
relative to the (moving) base — exactly what a camera-mounted tracker would
report for a tag that is actually fixed in the world as the base drives
toward or away from it.

This does not read TF or any real localization: it is pure /cmd_vel
integration, so it drifts like any open-loop dead-reckoning and is only
meant for bench-testing the reachability/grasp pipeline without a physical
tag, e.g. to drive gradient_base_controller / grasp.py off a synthetic
marker instead of `_marker_region()`'s fallback going stale.

Usage:
  ros2 run capacitynet mock_fiducial_marker --ros-args \
    -p marker_x:=0.5 -p marker_y:=0.0 -p marker_z:=0.3 -p marker_id:=0
"""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from ros2_markertracker_interfaces.msg import FiducialMarker, FiducialMarkerArray


class MockFiducialMarker(Node):
    """Dead-reckons a world-fixed tag pose against /cmd_vel and republishes it."""

    def __init__(self):
        super().__init__('mock_fiducial_marker')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('marker_topic', '/fiducial_markers')
        self.declare_parameter('base_frame', 'dsr01/base_link')
        self.declare_parameter('marker_id', 0)
        # Marker pose relative to base_frame *at startup* — this is the fixed
        # point in the world the mock tag sits at. Defaults match the manual
        # `ros2 topic pub` example this node replaces.
        self.declare_parameter('marker_x', 0.5)
        self.declare_parameter('marker_y', 0.0)
        self.declare_parameter('marker_z', 0.3)
        self.declare_parameter('marker_yaw', 0.0)
        self.declare_parameter('publish_rate', 10.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.marker_id = int(self.get_parameter('marker_id').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        # Marker's fixed pose in the anchor frame (= base_frame at t=0).
        self._marker_world = (
            float(self.get_parameter('marker_x').value),
            float(self.get_parameter('marker_y').value),
            float(self.get_parameter('marker_z').value),
            float(self.get_parameter('marker_yaw').value),
        )
        # Base pose in the anchor frame, dead-reckoned from /cmd_vel.
        self._base_x = 0.0
        self._base_y = 0.0
        self._base_theta = 0.0

        self._last_twist = Twist()
        self._last_tick = self.get_clock().now()

        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.marker_pub = self.create_publisher(
            FiducialMarkerArray, self.marker_topic, 10)

        period = 1.0 / max(publish_rate, 0.001)
        self.timer = self.create_timer(period, self._on_timer)

        mx, my, mz, myaw = self._marker_world
        self.get_logger().info(
            f'Mock fiducial marker id={self.marker_id} anchored at '
            f'({mx:+.3f}, {my:+.3f}, {mz:+.3f}, yaw={math.degrees(myaw):+.1f}deg) '
            f'relative to {self.base_frame} at startup')
        self.get_logger().info(
            f'  - Dead-reckoning from {self.cmd_vel_topic}, publishing '
            f'{self.marker_topic} at {publish_rate:.1f} Hz')

    def _on_cmd_vel(self, msg: Twist):
        self._last_twist = msg

    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now
        if dt > 0.0:
            self._integrate(dt)
        self._publish(now)

    def _integrate(self, dt):
        """Advance the dead-reckoned base pose by one step of the cached twist."""
        v = self._last_twist
        theta = self._base_theta
        # Twist is in the base frame; rotate into the anchor frame before
        # integrating position, same convention as a standard odometry node.
        self._base_x += (v.linear.x * math.cos(theta) - v.linear.y * math.sin(theta)) * dt
        self._base_y += (v.linear.x * math.sin(theta) + v.linear.y * math.cos(theta)) * dt
        self._base_theta += v.angular.z * dt

    def _publish(self, stamp):
        mx, my, mz, myaw = self._marker_world
        bx, by, theta = self._base_x, self._base_y, self._base_theta

        # Marker pose expressed in the (moved) base frame: inverse of the
        # base's pose composed with the marker's fixed anchor-frame pose.
        dx, dy = mx - bx, my - by
        local_x = dx * math.cos(theta) + dy * math.sin(theta)
        local_y = -dx * math.sin(theta) + dy * math.cos(theta)
        local_yaw = myaw - theta

        msg = FiducialMarkerArray()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = stamp.to_msg()
        msg.camera_frame_stamp = stamp.to_msg()

        marker = FiducialMarker()
        marker.id = self.marker_id
        marker.pose_cov_stamped.header.frame_id = self.base_frame
        marker.pose_cov_stamped.header.stamp = stamp.to_msg()
        pose = marker.pose_cov_stamped.pose.pose
        pose.position.x = local_x
        pose.position.y = local_y
        pose.position.z = mz
        pose.orientation.z = math.sin(local_yaw / 2.0)
        pose.orientation.w = math.cos(local_yaw / 2.0)
        # Covariance left at zero: on_markers() only reads pose.pose.
        msg.marker = [marker]

        self.marker_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockFiducialMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
