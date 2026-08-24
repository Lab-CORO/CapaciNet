#!/usr/bin/env python3
"""Move the mobile base toward better arm reachability.

Wiring only. The work is done by three components, each of which declares its
own parameters and creates its own ROS endpoints on this node:

    WorkspaceRegionSource  -> where to score (MPC goal + path tail, or marker)
    ReachabilityPipeline   -> grid_size**2 reachability maps -> Q_i -> grad Q  (CUDA group)
    BaseCommander          -> /cmd_vel + watchdog + interlocks    (control group)

Because they are independent, a node that leaves out the commander cannot move
the base — that is exactly what `workspace_probe` is, and it is how the pipeline
gets exercised on a bag without a robot attached.

This is the real-data counterpart of `gradient_controller_mock`, and replaces the
`enable_gradient_control` branch of `brain_orchestrator` (now off by default) so a
single node owns /cmd_vel.

Safety model: the node starts DISABLED and must be enabled explicitly, and it
disables itself whenever `grasp.py` reports a non-idle state.
"""

import signal

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from capacitynet.control.components.base_commander import BaseCommander
from capacitynet.control.components.control_debug_logger import ControlDebugLogger
from capacitynet.control.components.reachability_pipeline import ReachabilityPipeline
from capacitynet.control.components.workspace_region import WorkspaceRegionSource


class GradientBaseController(Node):
    """Reachability-gradient velocity controller for the mobile base."""

    def __init__(self):
        super().__init__('gradient_base_controller')

        # All CUDA work is confined to one group so exactly one inference runs at
        # a time. Everything else lives in the control group, which must keep
        # publishing while a ~440 ms inference is in flight — on a single-threaded
        # executor /cmd_vel would collapse to the inference rate (~2 Hz) and most
        # base drivers time out below ~4 Hz.
        self.cuda_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()

        self.region = WorkspaceRegionSource(self, callback_group=self.control_group)
        self.pipeline = ReachabilityPipeline(
            self, self.region, callback_group=self.cuda_group)
        self.commander = BaseCommander(
            self, self.pipeline.delta, callback_group=self.control_group)
        self.debug_logger = ControlDebugLogger(self, self.pipeline)

        # Wiring: the commander gates the pipeline, the pipeline feeds the
        # commander, a region that goes stale stops the base at once, and the
        # commander stops (independent of the gradient) once the MPC plan's
        # goal sphere and path-horizon sphere touch — read as "about to arrive".
        self.pipeline.gate = self.commander.allows_motion
        self.pipeline.on_result = self._on_result
        self.pipeline.on_skip = self.debug_logger.log_skip
        self.region.on_lost = self.commander.stop
        self.commander.region_converged = self.region.region_spheres_touching

        self._log_configuration()

    def _log_configuration(self):
        log = self.get_logger()
        log.info('Gradient base controller initialized')
        log.info(
            f'  - Voxel grid topic: {self.pipeline.voxel_grid_topic} '
            f'(cycle every {self.pipeline.cycle_period}s)')
        log.info(f'  - Marker topic: {self.region.marker_topic}')
        log.info(
            f'  - Region: goal={self.region.goal_topic} path={self.region.path_topic} '
            f'(horizon={self.region.path_horizon_s}s), fallback=marker')
        log.info(
            f'  - Radius: goal={self.region.workspace_radius}m '
            f'path={self.region.path_radius}m')
        log.info(f'  - Command topic: {self.commander.cmd_vel_topic}')
        log.info(
            f'  - Base speed: {self.commander.base_speed} m/s, '
            f'delta: {self.pipeline.delta} m, grid_size: {self.pipeline.grid_size} '
            f'({self.pipeline.grid_size ** 2} candidates)')
        log.info(f'  - Gradient method: {self.pipeline.gradient_method}')
        log.info(f'  - Backend: {self.pipeline.backend}')
        if self.region.static_workspace_center:
            log.info(
                f'  - Static workspace center: {self.region.static_workspace_center} '
                '(overrides MPC goal and marker)')
        log.info(
            '  - Convergence interlock: goal sphere touches path-horizon sphere '
            '-> stop base')
        log.info(
            f'  - Enabled: {self.commander.enabled} '
            f'(toggle via {self.get_name()}/enable)')

    def _on_result(self, result):
        """Act on one measurement: shape it into a command, then log the cycle."""
        (vx, vy), v_cap = self.commander.submit(
            result.gradient, result.cycle_s, result.score_center)
        self.debug_logger.log_cycle(
            result, self.region.source, self.commander, vx, vy, v_cap)

        if self.pipeline.log_timing:
            gx, gy = result.gradient
            self.get_logger().info(
                f'grad Q=({gx:+.4f},{gy:+.4f}) |{result.gradient_magnitude:.4f}| '
                f'v=({vx:+.4f},{vy:+.4f}) m/s '
                f'Q_c={result.score_center:.4f} '
                f'region={self.region.source}({result.region_size}) '
                f'cycle={result.cycle_s * 1000:.0f}ms cap={v_cap:.3f}m/s')

    def stop_base(self):
        """Best-effort stop, safe to call while the context is being torn down."""
        self.commander.shutdown()
        self.debug_logger.close()


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
