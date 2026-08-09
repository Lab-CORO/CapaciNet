#!/usr/bin/env python3
"""Plot Q(x, y) — reachability quality score — as an interpolated 3D surface.

By default the swept region is resolved the same way the production controller
resolves it: a live `WorkspaceRegionSource` component reads the MPC goal + the
tail of the predicted path (falling back to the fiducial marker), so the plotted
surface is the region actually scored on the robot — a union of spheres, not a
single point. `--center` bypasses all of that for quick hypothetical exploration
of one point with no goal/path/marker publisher needed.

Either way, ONE voxel grid message is then captured (live topic, or from a
`ros2 bag play`) and a dense grid of candidate base positions is swept around
that region — far denser than the 3x3 control grid — translating the region
mask by an exact number of voxels for every step (same trick as
gradient_controller.compute_quality_scores) so the surface is the true Q(x, y),
not a coarse 3x3 grid stretched by guesswork. A cubic spline then upsamples
that grid purely for display smoothness — it adds no new model evaluations.

Usage:
    # Live region resolution: needs a goal+path (or marker) publisher running,
    # e.g. `ros2 bag play <bag>` alongside a synthetic MPC goal publisher such
    # as the fake_mpc.py pattern used in this package's own bag tests.
    python3 scripts/plot_reachability_surface.py --bag rosbag2_2026_07_01-15_16_24

    # Same, with the region's own ROS params overridden (anything
    # WorkspaceRegionSource declares — goal_topic, path_tail_samples, ...):
    python3 scripts/plot_reachability_surface.py --bag <bag> \
        --ros-args -p path_tail_samples:=0 -p mpc_timeout:=5.0

    # Quick override: one hypothetical point, no goal/path/marker needed:
    python3 scripts/plot_reachability_surface.py \
        --bag <bag> --center 0.60 0.30 0.40 --radius 0.30

    # Denser sweep, wider extent, save without opening a window:
    python3 scripts/plot_reachability_surface.py --bag <bag> \
        --extent 0.6 --step 0.02 --out /tmp/q_surface.png
"""

import argparse
import os
import subprocess
import sys
import time

from curobo_msgs.msg import SparseVoxelGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

try:
    import matplotlib
    if '--show' not in sys.argv:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - registers the '3d' projection
except ImportError:
    print('ERROR: matplotlib not found.', file=sys.stderr)
    raise

from scipy.interpolate import RegularGridInterpolator

try:
    from capacitynet.obstacle_transformer import ObstacleMapTransformer
    from capacitynet.reachability_engine import ReachabilityEngine
    from capacitynet.workspace_evaluation import WorkspaceEvaluation
    from capacitynet.workspace_region import WorkspaceRegionSource
except ImportError:
    # Fallback for running the script without colcon install.
    _pkg_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'capacitynet')
    sys.path.insert(0, _pkg_dir)
    from obstacle_transformer import ObstacleMapTransformer
    from reachability_engine import ReachabilityEngine
    from workspace_evaluation import WorkspaceEvaluation
    from workspace_region import WorkspaceRegionSource


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--center', type=float, nargs=3, default=None, metavar=('X', 'Y', 'Z'),
                   help='Bypass live region resolution: sweep around this single point '
                        'instead (voxel grid frame). No goal/path/marker publisher needed.')
    p.add_argument('--radius', type=float, default=0.30,
                   help='Region sphere radius, only used with --center (default: 0.30); '
                        "in live mode the resolved region's own workspace_radius applies")
    p.add_argument('--extent', type=float, default=0.40,
                   help='Half-width of the swept (x, y) square in meters (default: 0.40)')
    p.add_argument('--step', type=float, default=0.04,
                   help='Sweep step in meters; snapped to a whole number of voxels '
                        '(default: 0.04)')
    p.add_argument('--interp-factor', type=int, default=4,
                   help='Cubic upsampling factor applied to the swept grid for display '
                        'only, no extra model evaluations (default: 4, 0 to disable)')
    p.add_argument('--batch-size', type=int, default=32,
                   help='Model inference batch size for the sweep (default: 32)')

    p.add_argument('--voxel-grid-topic', default='/curobo_trajectory_planner/voxel_grid_sparse')
    p.add_argument('--bag', default=None,
                   help='If set, play this bag once with `ros2 bag play` to feed the '
                        'region and the voxel grid, instead of assuming a live source')
    p.add_argument('--start-offset', type=float, default=0.0,
                   help='Skip this many seconds into the bag before playing (only with '
                        '--bag) — avoids capturing a transient early goal before the arm '
                        'has settled into its real approach (default: 0)')
    p.add_argument('--timeout', type=float, default=30.0,
                   help='Seconds to wait for the region to resolve and one voxel grid '
                        'message to arrive (default: 30)')

    p.add_argument('--model-config',
                   default='/home/ros2_ws/src/capacitynet/config/test_reach.yaml')
    p.add_argument('--fp16', action='store_true')
    p.add_argument('--use-static-obstacles', action='store_true')
    p.add_argument('--static-obstacles-yaml',
                   default='/home/ros2_ws/src/capacitynet/config/floor_world.yml')

    p.add_argument('--out', default='/tmp/q_surface.png', help='Where to save the figure')
    p.add_argument('--html-out', default=None,
                   help='Also save a self-contained interactive HTML version (rotate/zoom '
                        'in any browser, no Python/ipympl needed to replay it) — requires '
                        '`pip install plotly`')
    p.add_argument('--show', action='store_true', help='Open an interactive window')
    # Strip --ros-args and everything after it before argparse sees the argv,
    # so WorkspaceRegionSource's own parameters (goal_topic, path_tail_samples,
    # workspace_radius, ...) can be overridden the normal ROS way.
    return p.parse_args(remove_ros_args(sys.argv)[1:])


class _RegionCapture(Node):
    """Resolves the live region (or freezes --center) against one voxel grid message.

    In live mode both must line up: the region is only accepted once it resolves
    against the *same* voxel grid message's frame, so the swept occupancy and the
    scored region are never a mismatched pair from different moments.
    """

    def __init__(self, voxel_topic, override_center):
        super().__init__('plot_reachability_surface')
        self.voxel_msg = None
        self.centers = None
        self.source = 'override'

        self.region = None if override_center else WorkspaceRegionSource(self)
        self._override_center = override_center

        self._voxel_count = 0
        self._last_logged_goal = None
        self._last_heartbeat_t = time.monotonic()

        self.create_subscription(SparseVoxelGrid, voxel_topic, self._on_voxel, 1)

    def _on_voxel(self, msg):
        if self.centers is not None:
            return
        self._voxel_count += 1

        if self.region is None:
            self.voxel_msg = msg
            self.centers = [tuple(self._override_center)]
            return

        # Surface the raw MPC goal as it evolves during bag playback, so a
        # transient early goal (before the arm settles into its real
        # approach) is visible without a separate `ros2 topic echo`.
        goal = self.region._mpc_goal
        if goal is not None and goal != self._last_logged_goal:
            print(f'  [voxel #{self._voxel_count}] mpc_goal -> '
                  f'({goal[0]:+.3f}, {goal[1]:+.3f}, {goal[2]:+.3f})')
            self._last_logged_goal = goal

        resolved = self.region.resolve(msg.header.frame_id)
        if resolved is not None:
            self.voxel_msg = msg
            self.centers = resolved
            self.source = self.region.source
        elif time.monotonic() - self._last_heartbeat_t > 2.0:
            print(f'  [voxel #{self._voxel_count}] still waiting for region to resolve '
                  f'(mpc_goal={"seen" if goal is not None else "none yet"})')
            self._last_heartbeat_t = time.monotonic()


def capture_region_and_voxel_grid(voxel_topic, override_center, timeout_s):
    rclpy.init(args=sys.argv)
    node = _RegionCapture(voxel_topic, override_center)
    if node.region is None:
        print(f'Waiting up to {timeout_s:.0f}s for one message on {voxel_topic} ...')
    else:
        print(f'Waiting up to {timeout_s:.0f}s for the live region to resolve on '
              f'{voxel_topic} (goal={node.region.goal_topic}, '
              f'path={node.region.path_topic}, marker={node.region.marker_topic}) ...')
    deadline = time.monotonic() + timeout_s
    try:
        while node.centers is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        if node.centers is None:
            hint = ('No goal/path/marker publisher was ever seen — use --center for a '
                    'hypothetical sweep instead.' if node.region is not None else '')
            raise TimeoutError(
                f'Could not capture a scored region within {timeout_s:.0f}s. {hint}')
        radius = node.region.radius if node.region is not None else None
        centers_str = ', '.join(f'({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:+.3f})'
                                for c in node.centers)
        print(f'Region resolved: source={node.source} centers=[{centers_str}] '
              f'(after {node._voxel_count} voxel message(s))')
        return node.voxel_msg, node.centers, radius, node.source
    finally:
        node.destroy_node()
        rclpy.shutdown()


def sweep_quality_surface(msg, centers, radius, extent, step, interp_factor,
                          batch_size, model_config, fp16, use_static_obstacles,
                          static_obstacles_yaml):
    import torch

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f'Loading reachability model on {device}...')
    engine = ReachabilityEngine(model_config, fp16=fp16)
    print(f'Backend: {"TensorRT" if engine.trt_model is not None else "PyTorch"}')

    resolution = float(msg.resolution)
    sx, sy, sz = msg.size_x, msg.size_y, msg.size_z
    origin = msg.origin
    vg_info = {'origin': origin, 'resolution': resolution,
               'size_x': sx, 'size_y': sy, 'size_z': sz}

    # Snap the step to a whole number of voxels so every sweep position is an
    # exact integer translation of the base mask (same trick as
    # gradient_controller.compute_quality_scores) — the sweep is then exact,
    # not an approximation. Works identically for a union of several centers:
    # the base mask covers the whole region, and translating it rigidly is
    # exactly what happens when the base moves and goal+path shift together.
    step_voxels = max(1, int(round(step / resolution)))
    step = step_voxels * resolution
    n_steps = max(1, int(round(extent / step)))
    offsets_1d = np.arange(-n_steps, n_steps + 1) * step
    print(f'Sweeping {len(offsets_1d)}x{len(offsets_1d)} = {len(offsets_1d)**2} positions '
          f'(step={step:.3f}m = {step_voxels} voxel(s), extent=+/-{n_steps * step:.2f}m)')

    obstacle_transformer = ObstacleMapTransformer(
        resolution=resolution, device=device,
        static_obstacles_yaml=static_obstacles_yaml if use_static_obstacles else None,
    )
    voxel_map = engine.preprocess(msg.occupied_indices, sx, sy, sz)

    region_eval = WorkspaceEvaluation(centers, radius, device)
    base_mask = region_eval.region_mask(**vg_info)

    positions = [(dx, dy) for dy in offsets_1d for dx in offsets_1d]
    scores = np.zeros(len(positions), dtype=np.float32)

    t_start = time.time()
    for batch_start in range(0, len(positions), batch_size):
        batch = positions[batch_start:batch_start + batch_size]
        maps = torch.cat(
            [obstacle_transformer.transform(voxel_map, dx, dy, origin=origin)
             for dx, dy in batch], dim=0)
        rms = engine.predict_batch(maps)

        for i, (dx, dy) in enumerate(batch):
            di = -int(round(dx / resolution))
            dj = -int(round(dy / resolution))
            mask = base_mask if (di == 0 and dj == 0) else base_mask.translated(di, dj)
            scores[batch_start + i] = mask.compute_mean(rms[i])
        print(f'  {min(batch_start + batch_size, len(positions))}/{len(positions)}', end='\r')
    print(f'\nSweep done in {time.time() - t_start:.1f}s')

    q_grid = scores.reshape(len(offsets_1d), len(offsets_1d))  # [dy, dx]

    # Extract the 9 scores 3x3 around the center for gradient computation, in
    # GradientBasedController.grid_offsets() index order: row 0-2 is y=+delta,
    # row 3-5 is y=0, row 6-8 is y=-delta, so dy must be walked +1, 0, -1 (not
    # ascending) to match; dx stays -1, 0, +1 within each row.
    center_idx = len(offsets_1d) // 2
    has_neighbors = len(offsets_1d) >= 3 and 0 < center_idx < len(offsets_1d) - 1
    if has_neighbors:
        scores_9 = []
        for di in [1, 0, -1]:
            for dj in [-1, 0, 1]:
                scores_9.append(q_grid[center_idx + di, center_idx + dj])
        scores_9 = np.array(scores_9, dtype=np.float32)
    else:
        scores_9 = None

    if interp_factor and interp_factor > 1:
        # q_grid axis 0 is dy, axis 1 is dx (see the `positions` construction above).
        interp = RegularGridInterpolator((offsets_1d, offsets_1d), q_grid, method='cubic')
        fine = np.linspace(offsets_1d[0], offsets_1d[-1], len(offsets_1d) * interp_factor)
        fine_dx, fine_dy = np.meshgrid(fine, fine)
        q_grid = interp(np.stack([fine_dy, fine_dx], axis=-1))
        offsets_1d = fine

    return offsets_1d, q_grid, scores_9, step


def _compute_plot_annotations(offsets_1d, q_grid, centers, radius, scores_9, step):
    # Geometry shared by the matplotlib and Plotly renderers — the max-Q
    # point, center point, gradient arrow, and object-to-grasp indicator —
    # so that math lives in exactly one place instead of two that could
    # silently drift apart.
    floor = float(q_grid.min()) - 0.02 * (q_grid.max() - q_grid.min() or 1)
    best = np.unravel_index(np.argmax(q_grid), q_grid.shape)
    center_idx = len(offsets_1d) // 2
    q_center = float(q_grid[center_idx, center_idx])

    gradient = None
    if scores_9 is not None and step is not None:
        import torch

        from capacitynet.gradient_controller import GradientBasedController

        gc = GradientBasedController(workspace_radius=radius, grid_spacing=step,
                                     device='cpu')
        scores_torch = torch.tensor(scores_9, dtype=torch.float32)
        grad_x, grad_y = gc.compute_gradient(scores_torch)

        norm = np.sqrt(grad_x**2 + grad_y**2) or 1.0
        scale = 0.20  # Arrow length in meters
        gradient = {
            'grad_x': grad_x, 'grad_y': grad_y,
            'tip_x': grad_x / norm * scale, 'tip_y': grad_y / norm * scale,
        }

    # Object to grasp: centers[0] is already expressed relative to the
    # current base position (same frame as the swept offsets), so it can be
    # placed directly on the floor plane. It usually sits outside the swept
    # window (extent is a local fine-positioning scale, the approach
    # distance is not) — clip the indicator to the plot edge and report the
    # true distance rather than silently drawing nothing.
    obj_x, obj_y = float(centers[0][0]), float(centers[0][1])
    obj_dist = float(np.hypot(obj_x, obj_y))
    edge = float(offsets_1d[-1]) * 0.85
    reach_ratio = min(obj_dist, edge) / obj_dist if obj_dist > 0 else 0.0

    return {
        'floor': floor,
        'best_x': float(offsets_1d[best[1]]), 'best_y': float(offsets_1d[best[0]]),
        'best_q': float(q_grid[best]),
        'q_center': q_center,
        'gradient': gradient,
        'object': {'tip_x': obj_x * reach_ratio, 'tip_y': obj_y * reach_ratio,
                   'dist': obj_dist},
    }


def plot_surface(offsets_1d, q_grid, centers, radius, source, out_path, show,
                 scores_9=None, step=None):
    dx, dy = np.meshgrid(offsets_1d, offsets_1d)
    ann = _compute_plot_annotations(offsets_1d, q_grid, centers, radius, scores_9, step)

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(dx, dy, q_grid, cmap='viridis',
                           linewidth=0, antialiased=True, alpha=0.95)
    ax.contourf(dx, dy, q_grid, zdir='z', offset=ann['floor'], cmap='viridis', alpha=0.4)

    ax.scatter([ann['best_x']], [ann['best_y']], [ann['best_q']],
               color='red', s=60, depthshade=False, label=f"max Q={ann['best_q']:.4f}")
    ax.scatter([0], [0], [ann['q_center']],
               color='white', edgecolor='black', s=60, depthshade=False, label='center')

    if ann['gradient'] is not None:
        g = ann['gradient']
        ax.quiver(0, 0, ann['q_center'],
                  g['tip_x'], g['tip_y'], 0,
                  color='red', arrow_length_ratio=0.35, linewidth=3.5,
                  label=f"∇Q = ({g['grad_x']:+.3f}, {g['grad_y']:+.3f}) m⁻¹")
        # Mark the target position at the arrow head with a yellow star
        ax.scatter([g['tip_x']], [g['tip_y']], [ann['q_center'] + 0.01],
                   color='yellow', s=250, marker='*', edgecolors='red',
                   linewidth=1.5, depthshade=False, zorder=100)

    obj = ann['object']
    ax.plot([0, obj['tip_x']], [0, obj['tip_y']], [ann['floor'], ann['floor']],
            color='limegreen', linewidth=2.5, linestyle='--', zorder=90)
    ax.scatter([obj['tip_x']], [obj['tip_y']], [ann['floor']],
               color='limegreen', s=160, marker='^', edgecolors='darkgreen',
               linewidth=1.2, depthshade=False, zorder=91,
               label=f"objet à saisir (dist={obj['dist']:.2f}m)")

    goal = tuple(round(c, 3) for c in centers[0])
    tail_note = f' + {len(centers) - 1} path point(s)' if len(centers) > 1 else ''
    ax.set_xlabel('base offset x (m)')
    ax.set_ylabel('base offset y (m)')
    ax.set_zlabel('Q (mean reachability)')
    ax.set_title(f'Reachability quality Q(x, y) — source={source}, goal={goal}{tail_note}, '
                 f'radius={radius}m')
    fig.colorbar(surf, ax=ax, shrink=0.6, label='Q')
    ax.legend(loc='upper left')
    fig.tight_layout()

    fig.savefig(out_path, dpi=150)
    print(f'Saved: {out_path}')
    if show:
        plt.show()


def plot_surface_html(offsets_1d, q_grid, centers, radius, source, out_path,
                      scores_9=None, step=None):
    # Same figure as plot_surface(), as a self-contained interactive HTML file:
    # rotate/zoom/pan in any browser, no Python kernel or ipympl needed to
    # replay it later. Open it directly, or via VS Code's built-in
    # "Simple Browser: Show" command.
    try:
        import plotly.graph_objects as go
    except ImportError:
        print('ERROR: --html-out requires plotly (`pip install plotly`).', file=sys.stderr)
        raise

    ann = _compute_plot_annotations(offsets_1d, q_grid, centers, radius, scores_9, step)

    traces = [
        go.Surface(x=offsets_1d, y=offsets_1d, z=q_grid, colorscale='Viridis',
                   opacity=0.95, colorbar={'title': 'Q'}),
        go.Scatter3d(x=[ann['best_x']], y=[ann['best_y']], z=[ann['best_q']],
                     mode='markers', marker={'color': 'red', 'size': 5},
                     name=f"max Q={ann['best_q']:.4f}"),
        go.Scatter3d(x=[0], y=[0], z=[ann['q_center']], mode='markers',
                     marker={'color': 'white', 'size': 5,
                             'line': {'color': 'black', 'width': 2}},
                     name='center'),
    ]

    if ann['gradient'] is not None:
        g = ann['gradient']
        traces.append(go.Scatter3d(
            x=[0, g['tip_x']], y=[0, g['tip_y']], z=[ann['q_center']] * 2,
            mode='lines', line={'color': 'red', 'width': 6},
            name=f"∇Q = ({g['grad_x']:+.3f}, {g['grad_y']:+.3f}) m⁻¹"))
        traces.append(go.Scatter3d(
            x=[g['tip_x']], y=[g['tip_y']], z=[ann['q_center']], mode='markers',
            marker={'color': 'yellow', 'size': 8, 'symbol': 'diamond',
                    'line': {'color': 'red', 'width': 2}},
            showlegend=False))

    obj = ann['object']
    traces.append(go.Scatter3d(
        x=[0, obj['tip_x']], y=[0, obj['tip_y']], z=[ann['floor']] * 2,
        mode='lines', line={'color': 'limegreen', 'width': 5, 'dash': 'dash'},
        name=f"objet à saisir (dist={obj['dist']:.2f}m)"))
    traces.append(go.Scatter3d(
        x=[obj['tip_x']], y=[obj['tip_y']], z=[ann['floor']], mode='markers',
        marker={'color': 'limegreen', 'size': 7, 'symbol': 'diamond',
                'line': {'color': 'darkgreen', 'width': 2}},
        showlegend=False))

    goal = tuple(round(c, 3) for c in centers[0])
    tail_note = f' + {len(centers) - 1} path point(s)' if len(centers) > 1 else ''
    fig = go.Figure(data=traces)
    fig.update_layout(
        title=f'Reachability quality Q(x, y) — source={source}, goal={goal}{tail_note}, '
              f'radius={radius}m',
        scene={'xaxis_title': 'base offset x (m)', 'yaxis_title': 'base offset y (m)',
               'zaxis_title': 'Q (mean reachability)'},
        legend={'x': 0.02, 'y': 0.98})

    fig.write_html(out_path)
    print(f'Saved interactive HTML: {out_path}')


def main():
    args = parse_args()

    bag_proc = None
    if args.bag:
        bag_cmd = ['ros2', 'bag', 'play', args.bag]
        if args.start_offset > 0:
            bag_cmd += ['--start-offset', str(args.start_offset)]
            print(f'Playing bag: {args.bag} (starting {args.start_offset:.1f}s in)')
        else:
            print(f'Playing bag: {args.bag}')
        bag_proc = subprocess.Popen(bag_cmd,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        msg, centers, live_radius, source = capture_region_and_voxel_grid(
            args.voxel_grid_topic, args.center, args.timeout)
    finally:
        if bag_proc is not None:
            bag_proc.terminate()
            bag_proc.wait(timeout=5.0)

    radius = args.radius if args.center is not None else live_radius

    offsets_1d, q_grid, scores_9, step_used = sweep_quality_surface(
        msg, centers, radius, args.extent, args.step, args.interp_factor,
        args.batch_size, args.model_config, args.fp16,
        args.use_static_obstacles, args.static_obstacles_yaml)

    plot_surface(offsets_1d, q_grid, centers, radius, source, args.out, args.show,
                 scores_9=scores_9, step=step_used)

    if args.html_out:
        plot_surface_html(offsets_1d, q_grid, centers, radius, source, args.html_out,
                          scores_9=scores_9, step=step_used)


if __name__ == '__main__':
    main()
