#!/usr/bin/env python3
"""Latest voxel grid -> N*N reachability maps -> Q_i -> grad Q, every cycle_period.

    SparseVoxelGrid -> ReachabilityEngine.preprocess
                    -> ObstacleMapTransformer (grid_size x grid_size grid of shifted obstacle maps)
                    -> ReachabilityEngine.predict_batch  (grid_size**2 reachability maps)
                    -> GradientBasedController.evaluate  (Q_i, grad Q)

The voxel grid subscription only caches the latest message; a timer drives the
actual cycle at a fixed `cycle_period`, decoupling the (expensive, ~440 ms)
inference rate from whatever rate the voxel grid happens to publish at.

This component produces a *measurement*, never a command. Whoever wants to act
on it subscribes via `on_result`; whoever only wants to look at it (a probe, a
plotting node) does the same and simply never instantiates a BaseCommander.
"""

from dataclasses import dataclass, field
import time

import numpy as np

from curobo_msgs.msg import SparseVoxelGrid
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray

from capacitynet.control.scripts.gradient_controller import GradientBasedController
from capacitynet.control.scripts.grid_markers import build_grid_markers
from capacitynet.control.components.node_component import NodeComponent
from capacitynet.control.scripts.obstacle_transformer import ObstacleMapTransformer
from capacitynet.rm_model.reachability_engine import ReachabilityEngine


@dataclass
class GradientResult:
    """One control cycle's measurement, handed to `on_result`."""

    gradient: tuple            # (grad_x, grad_y) of Q, per meter
    gradient_magnitude: float
    scores: object             # numpy array of the grid_size**2 quality scores
    score_center: float        # Q at the untranslated (center) position
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
            callback_group: callback group for the voxel grid subscription and
                the cycle timer. Give this its own MutuallyExclusiveCallbackGroup
                so exactly one inference runs at a time and a ~440 ms cycle
                cannot stall the node's other callbacks.
        """
        super().__init__(node, prefix, callback_group)
        self.region = region

        self.voxel_grid_topic = self._declare(
            'voxel_grid_topic', '/curobo_trajectory_planner/voxel_grid_sparse')
        # Fixed cadence for the inference cycle, independent of how fast the
        # voxel grid actually publishes: the subscription only caches the
        # latest message, a timer runs the cycle on it every cycle_period.
        self.cycle_period = float(self._declare('cycle_period', 2.0))
        self.model_config_path = self._declare(
            'model_config_path', '/home/ros2_ws/src/capacitynet/config/test_reach.yaml')
        self.fp16 = bool(self._declare('fp16', False))
        self.grid_spacing = float(self._declare('grid_spacing', 0.10))
        # Candidate grid is grid_size x grid_size (must be odd, >= 3). Cost
        # scales linearly with grid_size**2 — see OPTIMIZATIONS.md. Validated
        # here rather than left to GradientBasedController so a bad value fails
        # before _setup_engine() spends seconds loading the model onto the GPU.
        self.grid_size = int(self._declare('grid_size', 3))
        if self.grid_size < 3 or self.grid_size % 2 == 0:
            raise ValueError(
                f'grid_size must be odd and >= 3, got {self.grid_size!r}')
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
        # Called with a short reason string when a cycle is skipped before
        # inference runs (gate closed, no voxel grid yet, region unresolved, or
        # the region doesn't fit the grid) — see _on_timer. Never called for a
        # failure *during* inference; only for these known, expected early-outs.
        self.on_skip = None

        # Latest voxel grid message, cached by the subscription callback and
        # consumed by the timer. None until the first message arrives.
        self._latest_msg = None

        # CPU numpy snapshot of the most recently completed cycle's n_candidates
        # reachability maps, obstacle maps (as actually fed to the model, i.e.
        # already shifted per candidate) and region masks — consumed by
        # ControlDebugLogger's save_reachability_maps service. None until the
        # first cycle completes. Overwritten every cycle regardless of whether
        # anything ever reads it, matching the existing _last_prediction_np
        # pattern in capacitynet.py.
        self.last_debug_dump = None

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
            grid_size=self.grid_size,
            device=self.device,
            gradient_method=self.gradient_method,
        )
        self.log.info('Reachability model loaded')

    def _setup_interfaces(self):
        node = self.node
        self.voxel_grid_sub = node.create_subscription(
            SparseVoxelGrid, self.voxel_grid_topic, self._on_voxel_grid, 1,
            callback_group=self.callback_group)
        # Same callback group as the subscription: MutuallyExclusiveCallbackGroup
        # already guarantees exactly one inference runs at a time, so a timer
        # tick and a subscription callback (which now only assigns a reference)
        # never race.
        self.cycle_timer = node.create_timer(
            self.cycle_period, self._on_timer, callback_group=self.callback_group)

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

    def _on_voxel_grid(self, msg: SparseVoxelGrid):
        """Cache the latest voxel grid; the timer drives the actual cycle."""
        self._latest_msg = msg

    def _skip(self, reason):
        """Report a cycle skipped before inference ran. See `on_skip`."""
        if self.on_skip is not None:
            self.on_skip(reason)

    def _on_timer(self):
        """Run one cycle on the latest cached voxel grid, every cycle_period."""
        if not self.gate():
            self._skip('gate_closed')
            return

        msg = self._latest_msg
        if msg is None:
            self._skip('no_voxel_grid')
            return

        centers = self.region.resolve(msg.header.frame_id)
        if centers is None:
            self._skip('region_unresolved')
            return

        sx, sy, sz = msg.size_x, msg.size_y, msg.size_z
        vg_info = {
            'origin': msg.origin,
            'resolution': float(msg.resolution),
            'size_x': sx, 'size_y': sy, 'size_z': sz,
        }
        # The scored centers range over +/- (grid_size//2)*delta in x and y
        # across the candidate grid.
        if not self.region.fits(
                centers, vg_info, extra_margin=self.delta * (self.grid_size // 2)):
            self._skip('region_out_of_grid')
            return

        t_start = time.time()

        voxel_map = self.engine.preprocess(msg.occupied_indices, sx, sy, sz)
        self.obstacle_transformer.update_resolution(vg_info['resolution'])
        transformed = self.obstacle_transformer.generate_grid_transforms(
            voxel_map, grid_spacing=self.delta, grid_size=self.grid_size,
            origin=vg_info['origin'])

        reachability_maps = self.engine.predict_batch(transformed)

        self.gradient_ctrl.update_workspace_region(
            centers, has_goal_center=self.region.region_has_goal)
        debug = self.gradient_ctrl.evaluate(reachability_maps, vg_info)

        # CPU snapshot for the debug dump service — cheap relative to the ~440 ms
        # inference cycle (a few ms to copy ~150 MB off a Jetson's shared DRAM),
        # so it is taken unconditionally rather than only when the service is
        # about to be called.
        self.last_debug_dump = {
            'reachability_maps': reachability_maps.cpu().numpy(),         # (n_candidates, D, D, D)
            'obstacle_maps': transformed.squeeze(1).cpu().numpy(),        # (n_candidates, D, D, D), per-candidate shifted input
            'masks': np.stack([m.to_numpy() for m in self.gradient_ctrl.last_masks]),  # (n_candidates, D, D, D) bool
            'scores': debug['scores'],
            'vg_info': vg_info,
            'frame_id': msg.header.frame_id,
        }

        del reachability_maps, transformed, voxel_map

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
            self.log.info('Q scores:\n' + self.gradient_ctrl.format_scores(result.scores))

        if result.mask_shift_exact is False:
            # delta is not a whole number of voxels, so the region masks cannot
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
            result.scores, result.gradient,
            self.gradient_ctrl.grid_offsets(), radius=self.grid_spacing/2,
            floor_z=self.floor_z))
        self.scores_pub.publish(
            Float32MultiArray(data=[float(s) for s in result.scores]))
