#!/usr/bin/env python3
"""Reachability pipeline without the base controller — an integration bench.

Same two components as `gradient_base_controller`, wired without a
`BaseCommander`. It therefore computes exactly the same Q_i and grad Q on the
same inputs, publishes the same debug topics, and *cannot* publish /cmd_vel:
there is no publisher for it anywhere in this node.

Use it to replay a bag, watch the region and the candidate-grid scores in
RViz, or plot Q over a run, with the robot safely out of the loop:

    ros2 run capacitynet workspace_probe --ros-args
        -p static_workspace_center:="0.6,0.3,0.4"

Every parameter of `gradient_base_controller` that concerns the region or the
inference works here unchanged; the motion parameters simply do not exist.
"""

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from capacitynet.control.components.reachability_pipeline import ReachabilityPipeline
from capacitynet.control.components.workspace_region import WorkspaceRegionSource


class WorkspaceProbe(Node):
    """Region + inference + Q, with no actuation."""

    def __init__(self):
        super().__init__('workspace_probe')

        self.cuda_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()

        self.region = WorkspaceRegionSource(self, callback_group=self.control_group)
        self.pipeline = ReachabilityPipeline(
            self, self.region, callback_group=self.cuda_group)
        # No commander: `gate` stays permissive and nothing is ever commanded.
        self.pipeline.on_result = self._on_result

        log = self.get_logger()
        log.info('Workspace probe initialized (no /cmd_vel publisher)')
        log.info(
            f'  - Voxel grid topic: {self.pipeline.voxel_grid_topic} '
            f'(cycle every {self.pipeline.cycle_period}s)')
        log.info(
            f'  - Region: goal={self.region.goal_topic} path={self.region.path_topic} '
            f'(tail={self.region.path_tail_samples}), fallback=marker')
        log.info(
            f'  - Radius: goal={self.region.radius}m path={self.region.path_radius}m, '
            f'delta: {self.pipeline.delta} m, grid_size: {self.pipeline.grid_size} '
            f'({self.pipeline.grid_size ** 2} candidates)')
        log.info(f'  - Backend: {self.pipeline.backend}')
        if self.region.static_workspace_center:
            log.info(
                f'  - Static workspace center: {self.region.static_workspace_center}')

    def _on_result(self, result):
        gx, gy = result.gradient
        self.get_logger().info(
            f'grad Q=({gx:+.4f},{gy:+.4f}) |{result.gradient_magnitude:.4f}| '
            f'Q_c={result.score_center:.4f} '
            f'region={self.region.source}({result.region_size}) '
            f'cycle={result.cycle_s * 1000:.0f}ms')


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceProbe()
    # Two threads for the same reason as the controller: the region callbacks
    # must keep running while a ~440 ms inference is in flight.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
