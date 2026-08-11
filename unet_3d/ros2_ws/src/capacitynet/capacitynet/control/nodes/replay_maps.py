#!/usr/bin/env python3
"""Replay saved reachability maps by stepping through HDF5 groups.

Publishes an incrementing index to /index at a fixed rate so that the
load_reachability_voxel node steps through /group/0, /group/1, … in the
HDF5 file and publishes each map to /reachability_map for RViz.

Usage:
  ros2 run capacitynet replay_maps --ros-args \
    -p total_maps:=50 -p rate:=1.0 -p loop:=false
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16


class ReplayMapsNode(Node):
    def __init__(self):
        super().__init__('replay_maps')

        self.declare_parameter('total_maps', 1)
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('loop', False)

        self._total = self.get_parameter('total_maps').value
        rate_hz = self.get_parameter('rate').value
        self._loop = self.get_parameter('loop').value

        self._current = 0
        self._pub = self.create_publisher(Int16, '/index', 1)

        period = 1.0 / max(rate_hz, 0.001)
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f'Replaying {self._total} maps at {rate_hz:.2f} Hz'
            + (' (looping)' if self._loop else '')
        )

    def _tick(self):
        if self._current >= self._total:
            if self._loop:
                self._current = 0
            else:
                self.get_logger().info('Replay finished.')
                self._timer.cancel()
                return

        msg = Int16()
        msg.data = self._current
        self._pub.publish(msg)
        self.get_logger().info(f'Index {self._current}/{self._total - 1}')
        self._current += 1


def main(args=None):
    rclpy.init(args=args)
    node = ReplayMapsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
