#!/usr/bin/env python3
"""Reachability quality score Q and its spatial gradient over a 3x3 candidate grid.

ROS-free by design: this module turns 9 reachability maps into a *measurement*
(Q_i and grad Q). Turning that measurement into a velocity command — gain,
saturation, taper, deadband — belongs to `base_commander`, so the same math can
be reused by a probe node that never touches /cmd_vel.
"""

import time

import numpy as np
import torch

from .workspace_evaluation import WorkspaceEvaluation


class GradientBasedController:
    """
    Reachability-gradient evaluator over a 3x3 grid of candidate base positions.

    Computes the quality-score gradient from a 3x3 grid of reachability maps.

    Grid layout (indices 0-8):
        0: (-δ,+δ)   1: (0,+δ)   2: (+δ,+δ)
        3: (-δ, 0)   4: (0, 0)   5: (+δ, 0)    δ = grid spacing (default 0.10m)
        6: (-δ,-δ)   7: (0,-δ)   8: (+δ,-δ)

    Each RM is computed in the robot base frame at that grid position.
    The workspace region moves relative to each base position.

    The region is one or more spheres, unioned: a single centre (radius
    `workspace_radius`) scores the ArTag alone, while goal + the tail of the MPC
    path scores the approach corridor the arm actually traverses — the goal
    sphere at `workspace_radius`, the path-tail spheres at `path_radius`. See
    compute_quality_scores.

    Gradient computation, selected by `gradient_method`:

        'central'       central finite differences, the minimal 4-point stencil:
                            ∂Q/∂x = (Q[5] - Q[3]) / (2δ)
                            ∂Q/∂y = (Q[1] - Q[7]) / (2δ)
                        Indices 0, 2, 6, 8 are inferred but never read.

        'least_squares' least-squares fit of a plane Q ≈ c + gx·x + gy·y over all
                        9 samples. The grid is symmetric (Σxᵢ = Σyᵢ = Σxᵢyᵢ = 0), so
                        the normal equations decouple to a closed form:
                            ∂Q/∂x = [(Q2+Q5+Q8) - (Q0+Q3+Q6)] / (6δ)
                            ∂Q/∂y = [(Q0+Q1+Q2) - (Q6+Q7+Q8)] / (6δ)
                        Exact for a locally planar Q (identical to 'central' in the
                        noise-free case), but with 3x lower variance under iid noise
                        on Q: σ²/(6δ²) vs σ²/(2δ²). Since the commanded direction is
                        ∇Q/|∇Q|, that direction stability is what damps hunting near
                        the optimum.

    Either way the centre Q[4] carries zero weight — it sits at the origin, so it
    defines the value, not the slope. It is reported in debug_info for convergence
    logging.
    """

    def __init__(self, workspace_radius=0.30, grid_spacing=0.10, device='cuda',
                 gradient_method='least_squares', path_radius=None):
        """
        Initialize the gradient evaluator.

        Args:
            workspace_radius: float, radius of the goal sphere (region centre 0)
                in meters (default: 0.30m)
            grid_spacing: float, spacing δ between grid points in meters (default: 0.10m)
            device: torch device ('cuda' or 'cpu')
            gradient_method: 'least_squares' (default, uses all 9 maps) or 'central'
            path_radius: float, radius of the path-tail spheres (region centres
                1+) in meters. None (default) reuses workspace_radius, matching
                the single-radius behavior before path_radius existed.
        """
        if gradient_method not in ('central', 'least_squares'):
            raise ValueError(
                f"gradient_method must be 'central' or 'least_squares', got {gradient_method!r}"
            )

        self.workspace_radius = workspace_radius
        self.path_radius = path_radius
        self.delta = grid_spacing
        self.device = device
        self.gradient_method = gradient_method

        # Workspace region: one or more centers in the world frame. A single
        # center is the ArTag/goal alone; several describe the union of spheres
        # around the goal plus the tail of the MPC path (see compute_quality_scores).
        self.workspace_centers_world = None

        # Region evaluator for the untranslated (index 4) position. It owns the
        # union mask and its cache, and is only rebuilt when the centers move, so
        # a static target rebuilds nothing between cycles.
        self._region_eval = None

        # Set per cycle: True when the 9 masks were obtained by integer shifts of
        # a single build, False when each had to be rebuilt (δ not a whole number
        # of voxels). Surfaced in debug_info so the node can log it.
        self.mask_shift_exact = None

        # Indices for gradient computation (central differences)
        self.idx_center = 4
        self.idx_x_plus = 5   # (+δ, 0)
        self.idx_x_minus = 3  # (-δ, 0)
        self.idx_y_plus = 1   # (0, +δ)
        self.idx_y_minus = 7  # (0, -δ)

        # Full columns/rows for the least-squares stencil. Column index maps to x
        # (col 0 = -δ, col 2 = +δ), row index maps to y (row 0 = +δ, row 2 = -δ) —
        # same convention as grid_offsets and ObstacleMapTransformer.
        self.idx_col_x_plus = (2, 5, 8)
        self.idx_col_x_minus = (0, 3, 6)
        self.idx_row_y_plus = (0, 1, 2)
        self.idx_row_y_minus = (6, 7, 8)

    def update_workspace_center(self, workspace_center_xyz):
        """
        Update the workspace center (ArTag position in world frame).

        Args:
            workspace_center_xyz: tuple (x, y, z) in meters
        """
        self.update_workspace_region([workspace_center_xyz])

    def update_workspace_region(self, centers_xyz):
        """
        Update the workspace region as a set of sphere centers in the world frame.

        Q is then the mean reachability over the *union* of the spheres, so a
        region covering the goal plus the last few MPC path poses scores the
        approach corridor the arm actually traverses rather than the tag alone.

        The region evaluator is only rebuilt when the centers actually move,
        preserving its mask cache across cycles with a static target.

        Args:
            centers_xyz: iterable of (x, y, z) tuples in meters
        """
        centers = [tuple(float(v) for v in c) for c in centers_xyz]
        if not centers:
            raise ValueError('Workspace region needs at least one center')
        if centers != self.workspace_centers_world:
            self.workspace_centers_world = centers
            self._region_eval = WorkspaceEvaluation(
                centers, self._radii_for(centers), self.device)

    def _radii_for(self, centers):
        """Per-centre radii: workspace_radius for the goal (index 0), path_radius
        (or workspace_radius if unset) for every path-tail centre.
        """
        path_r = self.path_radius if self.path_radius is not None else self.workspace_radius
        return [self.workspace_radius] + [path_r] * (len(centers) - 1)

    def grid_offsets(self):
        """
        The 9 base-position offsets of the candidate grid, in meters.

        Returns:
            list: 9 tuples (offset_x, offset_y), in grid-index order 0..8
        """
        offsets = []
        for index in range(9):
            row = index // 3  # 0, 1, 2 (top to bottom)
            col = index % 3   # 0, 1, 2 (left to right)
            offsets.append(((col - 1) * self.delta,   # -δ, 0, +δ
                            (1 - row) * self.delta))  # +δ, 0, -δ
        return offsets

    def _get_region_centers_for_rm(self, rm_index):
        """
        Get every region center in the reference frame of a specific RM.

        When the base moves by (offset_x, offset_y), the world — goal and path
        alike — moves by (-offset_x, -offset_y) in the base frame.

        Args:
            rm_index: int, index of the reachability map (0-8)

        Returns:
            list: (x, y, z) centers in the RM reference frame
        """
        if self.workspace_centers_world is None:
            raise RuntimeError(
                "Workspace region not set. Call update_workspace_region() first.")

        offset_x, offset_y = self.grid_offsets()[rm_index]
        return [(cx - offset_x, cy - offset_y, cz)
                for cx, cy, cz in self.workspace_centers_world]

    def compute_quality_scores(self, reachability_maps, vg_info):
        """
        Compute quality scores Q for all 9 reachability maps.

        Q_i is the mean reachability over the union of the region spheres,
        expressed in RM i's reference frame. Overlapping spheres are covered
        once, so Q is the mean over the swept volume, not an average of
        per-center means.

        The 9 regions are the same shape translated by the grid offsets. When δ
        spans a whole number of voxels (δ=0.10 m at 0.02 m resolution -> 5) they
        are exact integer translations, so the union is built once and shifted —
        N sphere constructions per cycle instead of 9·N. Otherwise each region is
        rebuilt from its own centers, which is slower but exact for any δ.

        Args:
            reachability_maps: torch.Tensor, shape (9, size_x, size_y, size_z) on GPU
            vg_info: dict with keys 'origin', 'resolution', 'size_x', 'size_y', 'size_z'

        Returns:
            torch.Tensor: Quality scores, shape (9,), on GPU
        """
        if self._region_eval is None:
            raise RuntimeError(
                "Workspace region not set. Call update_workspace_region() first.")

        resolution = vg_info['resolution']
        shift_voxels = self.delta / resolution
        shift_int = int(round(shift_voxels))
        self.mask_shift_exact = abs(shift_voxels - shift_int) < 1e-6 and shift_int != 0

        base_mask = self._region_eval.region_mask(**vg_info) if self.mask_shift_exact else None
        offsets = self.grid_offsets()

        scores = torch.zeros(9, dtype=reachability_maps.dtype, device=self.device)

        for i in range(9):
            if base_mask is not None:
                offset_x, offset_y = offsets[i]
                di = -int(round(offset_x / resolution))
                dj = -int(round(offset_y / resolution))
                mask = base_mask if (di == 0 and dj == 0) else base_mask.translated(di, dj)
            else:
                # δ is not a whole number of voxels: rebuild each region exactly.
                rm_centers = self._get_region_centers_for_rm(i)
                mask = WorkspaceEvaluation(
                    rm_centers, self._radii_for(rm_centers), self.device
                ).region_mask(**vg_info)

            scores[i] = mask.compute_mean(reachability_maps[i])

        return scores

    def compute_gradient(self, scores):
        """
        Compute the spatial gradient ∇Q from the 9 quality scores.

        Uses the stencil selected by `gradient_method` (see the class docstring).

        Args:
            scores: torch.Tensor, shape (9,), quality scores on GPU

        Returns:
            tuple: (grad_x, grad_y) gradient components (float)
        """
        if self.gradient_method == 'central':
            # Minimal 4-point stencil; diagonals unused.
            grad_x = (scores[self.idx_x_plus] - scores[self.idx_x_minus]) / (2 * self.delta)
            grad_y = (scores[self.idx_y_plus] - scores[self.idx_y_minus]) / (2 * self.delta)
        else:
            # Least-squares plane fit over all 9 samples. Σxᵢ² = Σyᵢ² = 6δ² on this
            # grid, hence the 6δ denominator.
            grad_x = (sum(scores[i] for i in self.idx_col_x_plus)
                      - sum(scores[i] for i in self.idx_col_x_minus)) / (6 * self.delta)
            grad_y = (sum(scores[i] for i in self.idx_row_y_plus)
                      - sum(scores[i] for i in self.idx_row_y_minus)) / (6 * self.delta)

        return grad_x.item(), grad_y.item()

    def evaluate(self, reachability_maps, vg_info):
        """
        Score the 9 candidate positions and fit the gradient.

        This is a measurement, not a command: no gain, no saturation, no message.

        Args:
            reachability_maps: torch.Tensor, shape (9, size_x, size_y, size_z) on GPU
            vg_info: dict with voxel grid parameters

        Returns:
            dict: scores, gradient, magnitude, centre score, region metadata, timing
        """
        t_start = time.time()

        scores = self.compute_quality_scores(reachability_maps, vg_info)
        t_after_scores = time.time()

        grad_x, grad_y = self.compute_gradient(scores)
        t_end = time.time()

        return {
            'scores': scores.cpu().numpy(),
            'gradient': (grad_x, grad_y),
            'gradient_magnitude': float(np.hypot(grad_x, grad_y)),
            'score_center': scores[self.idx_center].item(),
            'gradient_method': self.gradient_method,
            'region_size': len(self.workspace_centers_world),
            'mask_shift_exact': self.mask_shift_exact,
            'timing': {
                'score_computation': (t_after_scores - t_start) * 1000,
                'gradient_computation': (t_end - t_after_scores) * 1000,
                'total': (t_end - t_start) * 1000,
            },
        }
