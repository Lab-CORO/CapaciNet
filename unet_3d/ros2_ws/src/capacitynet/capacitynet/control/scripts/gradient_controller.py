#!/usr/bin/env python3
"""Reachability quality score Q and its spatial gradient over an NxN candidate grid.

ROS-free by design: this module turns N*N reachability maps into a *measurement*
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
    Reachability-gradient evaluator over an NxN grid of candidate base positions.

    Computes the quality-score gradient from an NxN grid of reachability maps
    (N = `grid_size`, odd, default 3 -> 9 candidates).

    Grid layout for grid_size=3 (indices 0-8), the default:
        0: (-δ,+δ)   1: (0,+δ)   2: (+δ,+δ)
        3: (-δ, 0)   4: (0, 0)   5: (+δ, 0)    δ = grid spacing (default 0.10m)
        6: (-δ,-δ)   7: (0,-δ)   8: (+δ,-δ)
    Larger odd grid_size extends the same lattice outward in steps of δ (see
    grid_offsets); the center index is always n_candidates // 2.

    Each RM is computed in the robot base frame at that grid position.
    The workspace region moves relative to each base position.

    The region is one or more spheres, unioned: a single centre (radius
    `workspace_radius`) scores the ArTag alone, while goal + the tail of the MPC
    path scores the approach corridor the arm actually traverses — the goal
    sphere at `workspace_radius`, the path-tail spheres at `path_radius`. See
    compute_quality_scores.

    Gradient computation, selected by `gradient_method`:

        'central'       central finite differences, the minimal 4-point stencil
                        using only the center's 4 orthogonal neighbors (always
                        ±δ away, regardless of grid_size):
                            ∂Q/∂x = (Q[center+1] - Q[center-1]) / (2δ)
                            ∂Q/∂y = (Q[center-N] - Q[center+N]) / (2δ)
                        Every other candidate is inferred (inference cost is
                        still paid for it) but never read — at grid_size>3 the
                        outer ring has no effect on the commanded direction.

        'least_squares' least-squares fit of a plane Q ≈ c + gx·x + gy·y over all
                        N*N samples. Any odd square lattice centered at the
                        origin is symmetric (Σxᵢ = Σyᵢ = Σxᵢyᵢ = 0), so the
                        normal equations always decouple to a closed form:
                            ∂Q/∂x = Σ(xᵢ·Qᵢ) / Σxᵢ²
                            ∂Q/∂y = Σ(yᵢ·Qᵢ) / Σyᵢ²
                        At grid_size=3 this is algebraically the classic
                        [(Q2+Q5+Q8)-(Q0+Q3+Q6)]/(6δ) form (Σxᵢ²=6δ² there), just
                        computed as a weighted sum so it generalizes to any N.
                        Exact for a locally planar Q (identical to 'central' in
                        the noise-free case at any grid_size), but with lower
                        variance under iid noise on Q, and that variance keeps
                        dropping as grid_size grows (more samples). Since the
                        commanded direction is ∇Q/|∇Q|, that direction stability
                        is what damps hunting near the optimum.

    Either way the centre carries zero weight — it sits at the origin, so it
    defines the value, not the slope. It is reported in debug_info for convergence
    logging.
    """

    def __init__(self, workspace_radius=0.30, grid_spacing=0.10, device='cuda',
                 gradient_method='least_squares', path_radius=None, grid_size=3):
        """
        Initialize the gradient evaluator.

        Args:
            workspace_radius: float, radius of the goal sphere (region centre 0)
                in meters (default: 0.30m)
            grid_spacing: float, spacing δ between grid points in meters (default: 0.10m)
            device: torch device ('cuda' or 'cpu')
            gradient_method: 'least_squares' (default, uses all N*N maps) or 'central'
            path_radius: float, radius of the path-tail spheres (region centres
                1+) in meters. None (default) reuses workspace_radius, matching
                the single-radius behavior before path_radius existed.
            grid_size: int, odd, >= 3 (default: 3). Candidate grid is
                grid_size x grid_size = n_candidates positions. Cost scales
                linearly with n_candidates (see OPTIMIZATIONS.md) — grid_size=5
                (25 candidates) costs ~2.5x today's default per cycle.
        """
        if gradient_method not in ('central', 'least_squares'):
            raise ValueError(
                f"gradient_method must be 'central' or 'least_squares', got {gradient_method!r}"
            )
        if grid_size < 3 or grid_size % 2 == 0:
            raise ValueError(f'grid_size must be odd and >= 3, got {grid_size!r}')

        self.workspace_radius = workspace_radius
        self.path_radius = path_radius
        self.delta = grid_spacing
        self.device = device
        self.gradient_method = gradient_method
        self.grid_size = grid_size
        self.n_candidates = grid_size * grid_size

        # Workspace region: one or more centers in the world frame. A single
        # center is the ArTag/goal alone; several describe the union of spheres
        # around the goal plus the tail of the MPC path (see compute_quality_scores).
        self.workspace_centers_world = None
        # Whether index 0 of workspace_centers_world is a true goal (gets
        # workspace_radius) or a path-tail point standing in for it when no
        # fresh mpc_goal exists (gets path_radius like the rest) — mirrors
        # WorkspaceRegionSource.region_has_goal, set via update_workspace_region.
        self._region_has_goal = True

        # Region evaluator for the untranslated (center) position. It owns the
        # union mask and its cache, and is only rebuilt when the centers move, so
        # a static target rebuilds nothing between cycles.
        self._region_eval = None

        # Set per cycle: True when the n_candidates masks were obtained by integer
        # shifts of a single build, False when each had to be rebuilt (δ not a
        # whole number of voxels). Surfaced in debug_info so the node can log it.
        self.mask_shift_exact = None

        # The n_candidates VoxelMask instances used by the most recent
        # compute_quality_scores() call, in candidate order. Not consumed here —
        # kept only so a debug consumer (ControlDebugLogger's reachability-map
        # dump) can save exactly the region mask that scored each candidate,
        # without rebuilding it.
        self.last_masks = None

        # Indices for the 'central' 4-point stencil: the center's 4 orthogonal
        # neighbors, always exactly ±δ away regardless of grid_size.
        self.idx_center = self.n_candidates // 2
        self.idx_x_plus = self.idx_center + 1        # (+δ, 0)
        self.idx_x_minus = self.idx_center - 1        # (-δ, 0)
        self.idx_y_plus = self.idx_center - grid_size  # (0, +δ)
        self.idx_y_minus = self.idx_center + grid_size  # (0, -δ)

        # Cached offset tensors for the 'least_squares' weighted-sum gradient
        # (see grid_offsets and the class docstring). Static given grid_size and
        # delta, so built once here rather than every evaluate() call.
        offsets = self.grid_offsets()
        self._offset_x = torch.tensor(
            [o[0] for o in offsets], dtype=torch.float32, device=device)
        self._offset_y = torch.tensor(
            [o[1] for o in offsets], dtype=torch.float32, device=device)
        self._sum_offset_x2 = float((self._offset_x ** 2).sum())
        self._sum_offset_y2 = float((self._offset_y ** 2).sum())

    def update_workspace_center(self, workspace_center_xyz):
        """
        Update the workspace center (ArTag position in world frame).

        Args:
            workspace_center_xyz: tuple (x, y, z) in meters
        """
        self.update_workspace_region([workspace_center_xyz])

    def update_workspace_region(self, centers_xyz, has_goal_center=True):
        """
        Update the workspace region as a set of sphere centers in the world frame.

        Q is then the mean reachability over the *union* of the spheres, so a
        region covering the goal plus the last few MPC path poses scores the
        approach corridor the arm actually traverses rather than the tag alone.

        The region evaluator is only rebuilt when the centers (or has_goal_center)
        actually change, preserving its mask cache across cycles with a static
        target.

        Args:
            centers_xyz: iterable of (x, y, z) tuples in meters
            has_goal_center: whether centers[0] is a true goal (gets
                workspace_radius) or a path-tail point standing in for it
                (gets path_radius like the rest) — see
                WorkspaceRegionSource.region_has_goal, which this normally
                mirrors 1:1.
        """
        centers = [tuple(float(v) for v in c) for c in centers_xyz]
        if not centers:
            raise ValueError('Workspace region needs at least one center')
        if centers != self.workspace_centers_world or has_goal_center != self._region_has_goal:
            self.workspace_centers_world = centers
            self._region_has_goal = has_goal_center
            self._region_eval = WorkspaceEvaluation(
                centers, self._radii_for(centers), self.device)

    def _radii_for(self, centers):
        """Per-centre radii: workspace_radius for the goal (index 0), path_radius
        (or workspace_radius if unset) for every path-tail centre. When the
        region has no true goal (has_goal_center=False), every centre gets
        path_radius, including index 0.
        """
        path_r = self.path_radius if self.path_radius is not None else self.workspace_radius
        if self._region_has_goal:
            return [self.workspace_radius] + [path_r] * (len(centers) - 1)
        return [path_r] * len(centers)

    def grid_offsets(self):
        """
        The n_candidates base-position offsets of the candidate grid, in meters.

        Returns:
            list: n_candidates tuples (offset_x, offset_y), in grid-index order
                0..n_candidates-1 (row-major, top to bottom, left to right)
        """
        n = self.grid_size
        half = n // 2
        offsets = []
        for index in range(n * n):
            row = index // n  # 0..n-1 (top to bottom)
            col = index % n   # 0..n-1 (left to right)
            offsets.append(((col - half) * self.delta,   # -half*δ .. +half*δ
                            (half - row) * self.delta))  # +half*δ .. -half*δ
        return offsets

    def _get_region_centers_for_rm(self, rm_index):
        """
        Get every region center in the reference frame of a specific RM.

        When the base moves by (offset_x, offset_y), the world — goal and path
        alike — moves by (-offset_x, -offset_y) in the base frame.

        Args:
            rm_index: int, index of the reachability map (0..n_candidates-1)

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
        Compute quality scores Q for all n_candidates reachability maps.

        Q_i is the mean reachability over the union of the region spheres,
        expressed in RM i's reference frame. Overlapping spheres are covered
        once, so Q is the mean over the swept volume, not an average of
        per-center means.

        The n_candidates regions are the same shape translated by the grid
        offsets. When δ spans a whole number of voxels (δ=0.10 m at 0.02 m
        resolution -> 5) they are exact integer translations, so the union is
        built once and shifted — N sphere constructions per cycle instead of
        n_candidates·N. Otherwise each region is rebuilt from its own centers,
        which is slower but exact for any δ.

        Args:
            reachability_maps: torch.Tensor, shape (n_candidates, size_x, size_y,
                size_z) on GPU
            vg_info: dict with keys 'origin', 'resolution', 'size_x', 'size_y', 'size_z'

        Returns:
            torch.Tensor: Quality scores, shape (n_candidates,), on GPU
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

        scores = torch.zeros(
            self.n_candidates, dtype=reachability_maps.dtype, device=self.device)
        masks = []

        for i in range(self.n_candidates):
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
            masks.append(mask)

        self.last_masks = masks
        return scores

    def compute_gradient(self, scores):
        """
        Compute the spatial gradient ∇Q from the n_candidates quality scores.

        Uses the stencil selected by `gradient_method` (see the class docstring).

        Args:
            scores: torch.Tensor, shape (n_candidates,), quality scores on GPU

        Returns:
            tuple: (grad_x, grad_y) gradient components (float)
        """
        if len(scores) != self.n_candidates:
            raise ValueError(
                f'scores has length {len(scores)}, expected n_candidates='
                f'{self.n_candidates} (grid_size={self.grid_size})')

        if self.gradient_method == 'central':
            # Minimal 4-point stencil; every other candidate unused.
            grad_x = (scores[self.idx_x_plus] - scores[self.idx_x_minus]) / (2 * self.delta)
            grad_y = (scores[self.idx_y_plus] - scores[self.idx_y_minus]) / (2 * self.delta)
        else:
            # General weighted least-squares plane fit (see class docstring):
            # grad_x = Σ(xᵢ·Qᵢ)/Σxᵢ², grad_y = Σ(yᵢ·Qᵢ)/Σyᵢ² — valid for any odd
            # square lattice centered at the origin, not just grid_size=3.
            grad_x = torch.sum(self._offset_x * scores) / self._sum_offset_x2
            grad_y = torch.sum(self._offset_y * scores) / self._sum_offset_y2

        return grad_x.item(), grad_y.item()

    def format_scores(self, scores):
        """Format the n_candidates quality scores as a grid_size x grid_size block.

        Args:
            scores: sequence of n_candidates quality scores, row-major (see
                grid_offsets)

        Returns:
            str: multi-line, 2-space indented, 4-decimal block
        """
        n = self.grid_size
        rows = ['  ' + '  '.join(f'{float(scores[r * n + c]):.4f}' for c in range(n))
                for r in range(n)]
        return '\n'.join(rows)

    def evaluate(self, reachability_maps, vg_info):
        """
        Score the n_candidates candidate positions and fit the gradient.

        This is a measurement, not a command: no gain, no saturation, no message.

        Args:
            reachability_maps: torch.Tensor, shape (n_candidates, size_x, size_y,
                size_z) on GPU
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
