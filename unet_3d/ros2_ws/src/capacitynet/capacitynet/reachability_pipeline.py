#!/usr/bin/env python3
"""Live voxel grid -> 9 reachability maps -> Q_i -> grad Q.

    SparseVoxelGrid -> ReachabilityEngine.preprocess
                    -> ObstacleMapTransformer (3x3 grid of shifted obstacle maps)
                    -> ReachabilityEngine.predict_batch  (9 reachability maps)
                    -> GradientBasedController.evaluate  (Q_i, grad Q)

This component produces a *measurement*, never a command. Whoever wants to act
on it subscribes via `on_result`; whoever only wants to look at it (a probe, a
plotting node) does the same and simply never instantiates a BaseCommander.
"""

from dataclasses import dataclass, field
import time

from curobo_msgs.msg import SparseVoxelGrid
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray

from .gradient_controller import GradientBasedController
from .grid_markers import build_grid_markers
from .node_component import NodeComponent
from .obstacle_transformer import ObstacleMapTransformer
from .reachability_engine import ReachabilityEngine


@dataclass
class GradientResult:
    """One control cycle's measurement, handed to `on_result`."""

    gradient: tuple            # (grad_x, grad_y) of Q, per meter
    gradient_magnitude: float
    scores: object             # numpy array of the 9 quality scores
    score_center: float        # Q at the untranslated position (index 4)
    centers: list              # region sphere centers, in `frame_id`
    region_size: int           # how many spheres the union covers
    mask_shift_exact: bool     # False when each region mask had to be rebuilt
    cycle_s: float             # wall-clock duration of the whole cycle
    frame_id: str              # frame the voxel grid (and the centers) live in
    stamp: object = field(default=None)   # voxel grid header stamp


class ReachabilityPipeline(NodeComponent):
    """Runs the reachability model on live voxel grids and reports grad Q."""

    def __init__(self, node, region, prefix='', callback_group=None):
        """Attach the pipeline to `node`, loading the reachability model.

        Args:
            node: host rclpy Node
            region: a WorkspaceRegionSource — supplies the centers and the radius
            prefix: optional parameter/topic prefix ('' keeps flat names)
            callback_group: callback group for the voxel grid subscription. Give
                this its own MutuallyExclusiveCallbackGroup so exactly one
                inference runs at a time and a ~440 ms cycle cannot stall the
                node's other callbacks.
        """
        super().__init__(node, prefix, callback_group)
        self.region = region

        self.voxel_grid_topic = self._declare(
            'voxel_grid_topic', '/curobo_trajectory_planner/voxel_grid_sparse')
        self.model_config_path = self._declare(
            'model_config_path', '/home/ros2_ws/src/capacitynet/config/test_reach.yaml')
        self.fp16 = bool(self._declare('fp16', False))
        self.grid_spacing = float(self._declare('grid_spacing', 0.10))
        self.gradient_method = self._declare('gradient_method', 'least_squares')
        self.use_static_obstacles = bool(self._declare('use_static_obstacles', False))
        self.static_obstacles_yaml = self._declare(
            'static_obstacles_yaml', '/home/ros2_ws/src/capacitynet/config/floor_world.yml')
        self.log_timing = bool(self._declare('log_timing', True))
        self.log_quality_scores = bool(self._declare('log_quality_scores', False))
        self.publish_debug = bool(self._declare('publish_debug_markers', True))
        # z height (in the voxel grid frame) the grid_markers heatmap is flattened
        # to, so it reads as a floor-level 2D map rather than floating at
        # candidate-query height. Default assumes z=0 is the floor; override to
        # match your setup (e.g. floor_world.yml's floor top sits at z=-0.05).
        self.floor_z = float(self._declare('floor_z', 0.0))

        # Gate consulted before every cycle. A controlling node points this at its
        # commander's interlock; left alone, the pipeline always runs.
        self.gate = lambda: True
        # Called with a GradientResult once a cycle completes.
        self.on_result = None

        self._setup_engine()
        self._setup_interfaces()

    # ── Setup ────────────────────────────────────────────────────────────────

    def _setup_engine(self):
        import torch
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.log.info(f'Loading reachability model on {self.device}...')
        self.engine = ReachabilityEngine(self.model_config_path, fp16=self.fp16)

        # Resolution is a placeholder; it is overwritten per message from the
        # live grid via update_resolution().
        self.obstacle_transformer = ObstacleMapTransformer(
            resolution=0.02,
            device=self.device,
            static_obstacles_yaml=(
                self.static_obstacles_yaml if self.use_static_obstacles else None),
        )
        self.gradient_ctrl = GradientBasedController(
            workspace_radius=self.region.radius,
            path_radius=self.region.path_radius,
            grid_spacing=self.grid_spacing,
            device=self.device,
            gradient_method=self.gradient_method,
        )
        self.log.info('Reachability model loaded')

    def _setup_interfaces(self):
        node = self.node
        self.voxel_grid_sub = node.create_subscription(
            SparseVoxelGrid, self.voxel_grid_topic, self.on_voxel_grid, 1,
            callback_group=self.callback_group)

        self.grid_marker_pub = None
        self.scores_pub = None
        if self.publish_debug:
            self.grid_marker_pub = node.create_publisher(
                MarkerArray, self._private('grid_markers'), 10)
            self.scores_pub = node.create_publisher(
                Float32MultiArray, self._private('quality_scores'), 10)

    # ── Public API ───────────────────────────────────────────────────────────

    @property
    def delta(self):
        """Candidate grid spacing δ in meters."""
        return self.gradient_ctrl.delta

    @property
    def backend(self):
        """Name of the backend the engine actually loaded: 'TensorRT' or 'PyTorch'."""
        return 'TensorRT' if self.engine.trt_model is not None else 'PyTorch'

    # ── Cycle ────────────────────────────────────────────────────────────────

    def on_voxel_grid(self, msg: SparseVoxelGrid):
        """Run one cycle: 9 reachability maps -> Q_i -> grad Q."""
        if not self.gate():
            return

        centers = self.region.resolve(msg.header.frame_id)
        if centers is None:
            return

        sx, sy, sz = msg.size_x, msg.size_y, msg.size_z
        vg_info = {
            'origin': msg.origin,
            'resolution': float(msg.resolution),
            'size_x': sx, 'size_y': sy, 'size_z': sz,
        }
        # The scored centers range over +/- delta in x and y across the 3x3 grid.
        if not self.region.fits(centers, vg_info, extra_margin=self.delta):
            return

        t_start = time.time()

        voxel_map = self.engine.preprocess(msg.occupied_indices, sx, sy, sz)
        self.obstacle_transformer.update_resolution(vg_info['resolution'])
        transformed = self.obstacle_transformer.generate_grid_transforms(
            voxel_map, grid_spacing=self.delta, origin=vg_info['origin'])

        rm_9 = self.engine.predict_batch(transformed)

        self.gradient_ctrl.update_workspace_region(centers)
        debug = self.gradient_ctrl.evaluate(rm_9, vg_info)

        del rm_9, transformed, voxel_map

        result = GradientResult(
            gradient=debug['gradient'],
            gradient_magnitude=debug['gradient_magnitude'],
            scores=debug['scores'],
            score_center=debug['score_center'],
            centers=centers,
            region_size=debug['region_size'],
            mask_shift_exact=debug['mask_shift_exact'],
            cycle_s=time.time() - t_start,
            frame_id=msg.header.frame_id,
            stamp=msg.header.stamp,
        )

        self._log_cycle(result)
        self._publish_debug(result)

        if self.on_result is not None:
            self.on_result(result)

    # ── Output ───────────────────────────────────────────────────────────────

    def _log_cycle(self, result):
        if self.log_quality_scores:
            s = result.scores
            self.log.info(
                'Q scores:\n'
                f'  {s[0]:.4f}  {s[1]:.4f}  {s[2]:.4f}\n'
                f'  {s[3]:.4f}  {s[4]:.4f}  {s[5]:.4f}\n'
                f'  {s[6]:.4f}  {s[7]:.4f}  {s[8]:.4f}')

        if result.mask_shift_exact is False:
            # delta is not a whole number of voxels, so the 9 region masks cannot
            # be integer-shifted copies and each is rebuilt from its own centers.
            self.log.warn(
                f'grid_spacing={self.delta} m is not a whole number of voxels — '
                f'rebuilding {result.region_size} spheres per grid position '
                'instead of shifting one mask',
                throttle_duration_sec=30.0)

    def _publish_debug(self, result):
        if self.grid_marker_pub is None:
            return
        self.grid_marker_pub.publish(build_grid_markers(
            result.frame_id, self.node.get_clock().now().to_msg(),
            result.centers[0], result.scores, result.gradient,
            self.gradient_ctrl.grid_offsets(), cell_size=self.delta,
            floor_z=self.floor_z))
        self.scores_pub.publish(
            Float32MultiArray(data=[float(s) for s in result.scores]))
