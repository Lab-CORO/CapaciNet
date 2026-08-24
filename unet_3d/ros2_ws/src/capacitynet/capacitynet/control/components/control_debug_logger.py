#!/usr/bin/env python3
"""Fixed-schema CSV log of one gradient-control cycle's debug variables.

Complements the `log.info` line already printed per cycle (in the controlling
node's `_on_result`) with a row a plotting/analysis script can load directly —
gradient/Q, the commanded velocity, and every interlock `allows_motion()`
depends on, so a run where /cmd_vel stayed at zero can be diagnosed after the
fact instead of only live, from the logs.

Timing is limited to `cycle_s` (the pipeline's own total) and the wall-clock
interval between calls to `log_cycle`: no per-stage breakdown (preprocess vs.
predict vs. gradient) is available past ReachabilityPipeline's own boundary.

`q_center` is the mean reachability over the *union* of every region sphere.
The `ws_mean_0`..`ws_mean_{N-1}` columns (N = 1 goal + 1 path-horizon point,
fixed at construction so the CSV schema does not depend on which region source is
active this cycle) break that union back down into one mean per sphere,
computed independently — so an approach where the goal is reachable but the
path corridor is not (or vice versa) is visible even though it would average
out in q_center alone. Same order as GradientResult.centers; a cycle with
fewer spheres than N leaves the trailing columns blank, not zero.

`log_skip` writes a row for a cycle the pipeline never ran inference for at
all (gate closed, no voxel grid yet, region unresolved, or the region doesn't
fit the grid — see ReachabilityPipeline.on_skip) — only `timestamp` and
`skip_reason` are filled, so a gap in grad_x/q_center/etc. is distinguishable
from "the controller just hadn't ticked yet" instead of silently reading as a
missing row.

Also owns `~/save_reachability_maps` (std_srvs/Trigger), the HDF5 counterpart
of `/reachability_node/save_map` in capacitynet.py: instead of one dense map,
it dumps all n_candidates reachability maps of the most recently completed
cycle together with the per-candidate obstacle map actually fed to the model
and the region mask used to score it, so a candidate's Q can be reproduced
offline from exactly the three arrays that produced it. Each call writes a
new, timestamped file rather than appending, so a run can be replayed cycle
by cycle without one growing file to manage.

Also, opt-in via `debug_gradient_html_path`, accumulates n_candidates
(goal_x, goal_y, goal_z, Q) points per cycle — one per candidate the gradient
was actually fit from, not an interpolation. goal_x/goal_y for candidate i is
`result.centers[0]` (the goal's position in the voxel-grid frame that cycle,
relative to the base at that instant) offset by that candidate's own
`grid_offsets()` entry, same convention as
`GradientBasedController._get_region_centers_for_rm`; goal_z is that same
center's z, unmodified (candidates only translate in x/y, so every candidate
in a cycle shares one real height); Q is `result.scores[i]`. Candidate
`idx_center` (offset 0,0) is the position the base actually occupied that
cycle — the others are real model evaluations too (translated obstacle maps,
not spline-interpolated values), just at hypothetical positions never
physically visited. In the written HTML, the scatter's Z axis is that real
goal_z (color encodes Q instead), while the fitted surface's Z is Q rescaled
into the observed goal_z range so both traces share one axis — see
`_write_gradient_exploration_html`. `~/save_gradient_exploration`
(std_srvs/Trigger) writes everything accumulated so far as an interactive
Plotly HTML scatter + fitted surface — callable mid-run, any number of times,
like `~/save_reachability_maps`.
"""

import csv
import os
import time

import h5py
import numpy as np
from std_srvs.srv import Trigger

from capacitynet.control.components.node_component import NodeComponent

_FIELDS = [
    'timestamp', 'cycle_s', 'dt_since_last_s',
    'grad_x', 'grad_y', 'grad_mag', 'q_center',
    'region_source', 'region_size', 'mask_shift_exact',
    'vx_cmd', 'vy_cmd', 'v_cap',
    'enabled', 'converged', 'grasper_state',
    'grasper_allows_motion', 'region_converged', 'allows_motion',
    'skip_reason',
]


class ControlDebugLogger(NodeComponent):
    """Writes one CSV row per gradient-control cycle. Opt-in via `debug_csv_path`.

    Disabled (no file created) when `debug_csv_path` is left at its default
    empty string, so a production launch does not silently start writing
    files. Set it via a launch arg or `--ros-args -p debug_csv_path:=...` to
    enable.
    """

    def __init__(self, node, pipeline, prefix='', callback_group=None):
        """
        Args:
            node: host rclpy Node
            pipeline: the ReachabilityPipeline whose `last_debug_dump` backs
                save_reachability_maps — same wiring as BaseCommander(node,
                pipeline.delta, ...): a sibling component read at construction.
            prefix: optional parameter/topic prefix ('' keeps flat names)
            callback_group: callback group for the service
        """
        super().__init__(node, prefix, callback_group)
        self.pipeline = pipeline

        self.csv_path = self._declare('debug_csv_path', '')

        self._file = None
        self._writer = None
        self._last_stamp = None
        self._last_result = None
        self._last_region_source = None

        # Upper bound on how many region spheres a cycle can ever report: the
        # goal (if any) plus the single path-horizon point. Fixed at
        # construction time so the CSV schema is stable even though a given
        # cycle's actual region_size varies (marker/static/path_only = 1,
        # mpc_goal+path = up to this).
        self._max_region_size = 1 + (1 if self.pipeline.region.path_horizon_s >= 0.0 else 0)
        self._fields = _FIELDS + [f'ws_mean_{i}' for i in range(self._max_region_size)]

        if self.csv_path:
            path = os.path.expanduser(self.csv_path)
            os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
            self._file = open(path, 'w', newline='')
            self._writer = csv.DictWriter(self._file, fieldnames=self._fields)
            self._writer.writeheader()
            self._file.flush()
            self.log.info(f'ControlDebugLogger: writing to {path}')
        else:
            self.log.info(
                'ControlDebugLogger: disabled (set debug_csv_path to enable)')

        self.h5_dir = self._declare(
            'debug_h5_dir', '/home/ros2_ws/src/capacitynet/data')
        self.save_maps_srv = self.node.create_service(
            Trigger, self._private('save_reachability_maps'),
            self.handle_save_reachability_maps, callback_group=self.callback_group)

        self.gradient_html_path = self._declare('debug_gradient_html_path', '')
        self._explored_x = []
        self._explored_y = []
        self._explored_z = []
        self._explored_q = []
        if self.gradient_html_path:
            self.log.info(
                'ControlDebugLogger: accumulating explored-gradient points; call '
                f"{self._private('save_gradient_exploration')} to write "
                f'{self.gradient_html_path}')
        self.save_gradient_srv = self.node.create_service(
            Trigger, self._private('save_gradient_exploration'),
            self.handle_save_gradient_exploration, callback_group=self.callback_group)

    def log_cycle(self, result, region_source, commander, vx, vy, v_cap):
        """Write one row. No-op when disabled.

        Args:
            result: GradientResult produced by this cycle
            region_source: WorkspaceRegionSource.source at the time of this cycle
            commander: the BaseCommander wired to the same pipeline
            vx, vy, v_cap: BaseCommander.submit()'s return values for this cycle
        """
        # Cached regardless of whether the CSV is enabled: save_reachability_maps
        # uses these to enrich the HDF5 dump with the same cycle's Q/gradient.
        self._last_result = result
        self._last_region_source = region_source

        if self.gradient_html_path:
            goal_x, goal_y, goal_z = result.centers[0]
            offsets = self.pipeline.gradient_ctrl.grid_offsets()
            for (offset_x, offset_y), q in zip(offsets, result.scores):
                # Same convention as GradientBasedController._get_region_centers_for_rm:
                # candidate i's goal position, expressed in that candidate's own
                # (translated) frame, is the untranslated goal minus its offset.
                # z is not offset (candidates only translate in x/y), so every
                # candidate this cycle shares the same real height.
                self._explored_x.append(goal_x - offset_x)
                self._explored_y.append(goal_y - offset_y)
                self._explored_z.append(goal_z)
                self._explored_q.append(float(q))

        if self._writer is None:
            return

        now = self.node.get_clock().now()
        dt = (None if self._last_stamp is None
              else (now - self._last_stamp).nanoseconds * 1e-9)
        self._last_stamp = now

        gx, gy = result.gradient
        row = {
            'timestamp': now.nanoseconds * 1e-9,
            'cycle_s': round(result.cycle_s, 4),
            'dt_since_last_s': round(dt, 4) if dt is not None else '',
            'grad_x': round(gx, 5),
            'grad_y': round(gy, 5),
            'grad_mag': round(result.gradient_magnitude, 5),
            'q_center': round(result.score_center, 5),
            'region_source': region_source,
            'region_size': result.region_size,
            'mask_shift_exact': result.mask_shift_exact,
            'vx_cmd': round(vx, 5),
            'vy_cmd': round(vy, 5),
            'v_cap': round(v_cap, 5),
            'enabled': commander.enabled,
            'converged': commander.converged,
            'grasper_state': commander.grasper_state or '',
            'grasper_allows_motion': commander.grasper_allows_motion,
            'region_converged': commander.region_converged(),
            'allows_motion': commander.allows_motion(),
        }
        # One column per region sphere, in the same order as result.centers
        # (index 0 is the goal iff region_has_goal — see WorkspaceRegionSource).
        # Cycles with fewer spheres than _max_region_size leave the trailing
        # columns blank (DictWriter's restval), not zero, so an unused sphere
        # is distinguishable from one that genuinely scored 0.0.
        for i, q in enumerate(result.per_sphere_scores):
            row[f'ws_mean_{i}'] = round(q, 5)
        self._writer.writerow(row)
        self._file.flush()

    def log_skip(self, reason):
        """Write one CSV row for a cycle skipped before inference ran. No-op when disabled.

        Only `timestamp` and `skip_reason` are filled; every other column is
        blank since no GradientResult exists for this cycle. `dt_since_last_s`
        on the *next* successful row is left to show the gap, rather than
        resetting `_last_stamp` here — a skip is exactly what should make that
        gap visible.

        Args:
            reason: short machine-readable string — see ReachabilityPipeline.on_skip
        """
        if self._writer is None:
            return

        now = self.node.get_clock().now()
        self._writer.writerow({
            'timestamp': now.nanoseconds * 1e-9,
            'skip_reason': reason,
        })
        self._file.flush()

    # ── Reachability-map HDF5 dump ──────────────────────────────────────────

    def _save_reachability_maps(self, dir_path):
        """Write one new timestamped H5 file with all n_candidates (RM, obstacle,
        mask) triples for the most recently completed cycle.

        Each candidate's three arrays are saved together specifically because
        they must be read back together: obstacle_map is the input that was fed
        to the model to produce reachability_map, and mask is the region that was
        applied to reachability_map to get score — none of the three means
        anything on its own without the other two.

        Also saves, per candidate, masked_reachability_map (reachability_map with
        every voxel outside `mask` zeroed, so the scored region is visible without
        recombining the two arrays) and, once at the file level, q_scores — the
        n_candidates mean-inside-mask values (mask.compute_mean(reachability_map)
        in gradient_controller.py), the same numbers already duplicated per
        candidate as the `score` attr, gathered here into one array for a
        plotting script that wants all nine without walking every subgroup.

        One file per call (rather than one growing file with an appended group
        per call) so a run can be replayed cycle by cycle as a plain directory
        listing, and so a save mid-run never has to reopen/lock a file another
        process might be reading.
        """
        dump = self.pipeline.last_debug_dump
        vg_info = dump['vg_info']
        origin = vg_info['origin']
        offsets = self.pipeline.gradient_ctrl.grid_offsets()

        dir_path = os.path.expanduser(dir_path)
        os.makedirs(dir_path, exist_ok=True)
        filename = f'reachability_maps_{time.time():.6f}.h5'
        path = os.path.join(dir_path, filename)

        with h5py.File(path, 'w') as f:
            f.attrs['voxel_size'] = float(vg_info['resolution'])
            f.attrs['origin_x'] = float(origin.x)
            f.attrs['origin_y'] = float(origin.y)
            f.attrs['origin_z'] = float(origin.z)
            f.attrs['voxel_grid_size_x'] = float(vg_info['size_x'])
            f.attrs['voxel_grid_size_y'] = float(vg_info['size_y'])
            f.attrs['voxel_grid_size_z'] = float(vg_info['size_z'])
            f.attrs['frame_id'] = dump['frame_id']
            f.attrs['grid_size'] = self.pipeline.grid_size
            f.attrs['grid_spacing'] = self.pipeline.delta
            if self._last_result is not None:
                gx, gy = self._last_result.gradient
                f.attrs['grad_x'] = gx
                f.attrs['grad_y'] = gy
                f.attrs['grad_mag'] = self._last_result.gradient_magnitude
                f.attrs['q_center'] = self._last_result.score_center
                f.attrs['region_size'] = self._last_result.region_size
                f.attrs['region_source'] = self._last_region_source or ''

            n_candidates = dump['reachability_maps'].shape[0]
            f.create_dataset(
                'q_scores', data=np.asarray(dump['scores'], dtype=np.float64))

            for i in range(n_candidates):
                rm = dump['reachability_maps'][i].astype(np.float64)
                mask = dump['masks'][i]

                cgrp = f.create_group(f'candidate/{i}')
                cgrp.create_dataset('reachability_map', data=rm)
                cgrp.create_dataset(
                    'obstacle_map', data=dump['obstacle_maps'][i].astype(np.float64))
                cgrp.create_dataset('mask', data=mask)
                # Region actually scored: rm outside the mask contributed nothing
                # to Q, zeroed here so it reads directly without recombining
                # reachability_map and mask.
                cgrp.create_dataset('masked_reachability_map', data=rm * mask)
                cgrp.attrs['score'] = float(dump['scores'][i])
                cgrp.attrs['offset_x'] = float(offsets[i][0])
                cgrp.attrs['offset_y'] = float(offsets[i][1])

        return path

    def handle_save_reachability_maps(self, request, response):
        if self.pipeline.last_debug_dump is None:
            response.success = False
            response.message = 'No inference cycle completed yet.'
            return response
        try:
            path = self._save_reachability_maps(self.h5_dir)
            n = self.pipeline.grid_size ** 2
            self.log.info(f'Saved {n} reachability maps to {path}')
            response.success = True
            response.message = f'Saved to {path}'
        except Exception as e:
            self.log.error(f'Failed to save reachability maps HDF5: {e}')
            response.success = False
            response.message = str(e)
        return response

    # Minimum points to fit the 6-term quadratic without the normal equations
    # going singular/degenerate; a couple of cycles' margin above the 6 unknowns.
    _MIN_POINTS_FOR_SURFACE_FIT = 8

    def _fit_quadratic_surface(self, x, y, z, grid_n=30):
        """Least-squares fit Q(x, y) = a + bx + cy + dx^2 + ey^2 + fxy.

        Unlike plot_reachability_surface.py's dense regular sweep (interpolated
        with a cubic spline, no extrapolation), these points are scattered and
        irregular — one per control cycle, wherever the base happened to be —
        so there is no grid to interpolate. A global quadratic fit is the
        cheapest surface that still captures a local max/min, and it is defined
        everywhere, including past the convex hull of the samples.

        Returns:
            (grid_x, grid_y, grid_z): 2D arrays (grid_n x grid_n) ready for
                go.Surface, spanning the samples' bounding box with a 10% margin.
        """
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        z = np.asarray(z, dtype=np.float64)

        design = np.column_stack([np.ones_like(x), x, y, x**2, y**2, x * y])
        coeffs, *_ = np.linalg.lstsq(design, z, rcond=None)

        x_margin = 0.1 * (x.max() - x.min() or 1.0)
        y_margin = 0.1 * (y.max() - y.min() or 1.0)
        grid_x_1d = np.linspace(x.min() - x_margin, x.max() + x_margin, grid_n)
        grid_y_1d = np.linspace(y.min() - y_margin, y.max() + y_margin, grid_n)
        grid_x, grid_y = np.meshgrid(grid_x_1d, grid_y_1d)

        a, b, c, d, e, f = coeffs
        grid_z = (a + b * grid_x + c * grid_y
                  + d * grid_x**2 + e * grid_y**2 + f * grid_x * grid_y)
        return grid_x, grid_y, grid_z

    def _write_gradient_exploration_html(self, path):
        """Write the accumulated (goal_x, goal_y, goal_z, Q) points as Plotly scatter + surface.

        The scatter's Z axis is the real goal height (goal_z, unmodified —
        candidates only translate in x/y, so all n_candidates points in one
        cycle share the same z; it can still vary cycle to cycle if the goal
        itself moves in height), color-encoding Q instead so the reachability
        signal is not lost. The fitted quadratic surface (`_fit_quadratic_surface`,
        computed in (x, y, Q) space) is then linearly rescaled from Q into the
        observed goal_z range so it shares the same Z axis as the points instead
        of being plotted on an unrelated 0-1 scale; its own coloring
        (`surfacecolor`) still shows the true Q value, only its Z position is
        rescaled. With fewer than `_MIN_POINTS_FOR_SURFACE_FIT` points, only the
        markers are written.
        """
        import plotly.graph_objects as go

        path = os.path.expanduser(path)
        os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)

        n_points = len(self._explored_q)
        n_candidates = self.pipeline.grid_size ** 2
        traces = [go.Scatter3d(
            x=self._explored_x, y=self._explored_y, z=self._explored_z,
            mode='markers', name='explored candidates',
            marker={'size': 4, 'color': self._explored_q, 'colorscale': 'Viridis',
                    'colorbar': {'title': 'Q (mean reachability)'}},
        )]

        fit_note = ''
        if n_points >= self._MIN_POINTS_FOR_SURFACE_FIT:
            grid_x, grid_y, grid_q = self._fit_quadratic_surface(
                self._explored_x, self._explored_y, self._explored_q)

            z_min, z_max = min(self._explored_z), max(self._explored_z)
            q_min, q_max = min(self._explored_q), max(self._explored_q)
            q_span = (q_max - q_min) or 1.0
            grid_z = z_min + (grid_q - q_min) / q_span * (z_max - z_min)

            traces.append(go.Surface(
                x=grid_x, y=grid_y, z=grid_z, surfacecolor=grid_q,
                name='quadratic fit (Q, rescaled to goal_z range)',
                colorscale='Viridis', opacity=0.5, showscale=False))
        else:
            fit_note = (f' (no surface fit: needs >= {self._MIN_POINTS_FOR_SURFACE_FIT} '
                        f'points, have {n_points})')

        n_cycles = n_points // n_candidates
        fig = go.Figure(data=traces)
        fig.update_layout(
            title=f'Explored Q(goal_x, goal_y) — {n_candidates} candidates x '
                  f'{n_cycles} cycles{fit_note}',
            scene={'xaxis_title': 'goal offset x (m, voxel-grid frame)',
                   'yaxis_title': 'goal offset y (m, voxel-grid frame)',
                   'zaxis_title': 'z (m, real goal height) — surface Z is Q '
                                  'rescaled to this range'})
        fig.write_html(path)

    def handle_save_gradient_exploration(self, request, response):
        if not self.gradient_html_path:
            response.success = False
            response.message = 'debug_gradient_html_path is not set.'
            return response
        if not self._explored_q:
            response.success = False
            response.message = 'No cycles logged yet.'
            return response
        try:
            self._write_gradient_exploration_html(self.gradient_html_path)
            n = len(self._explored_q)
            self.log.info(f'Saved {n} explored-gradient points to {self.gradient_html_path}')
            response.success = True
            response.message = f'Saved {n} points to {self.gradient_html_path}'
        except ImportError:
            msg = 'plotly is not installed (`pip install plotly`)'
            self.log.error(f'Failed to save gradient exploration HTML: {msg}')
            response.success = False
            response.message = msg
        except Exception as e:
            self.log.error(f'Failed to save gradient exploration HTML: {e}')
            response.success = False
            response.message = str(e)
        return response

    def close(self):
        """Flush and close the CSV file, if one is open. Safe to call more than once."""
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
            self.log.info('ControlDebugLogger: CSV closed')
