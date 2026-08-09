#!/usr/bin/env python3

import torch
import numpy as np
import time
from geometry_msgs.msg import Twist
from .voxel_mask import VoxelMask


class GradientBasedController:
    """
    Gradient-based mobile base controller using reachability maps.

    Computes quality score gradient from a 3x3 grid of reachability maps
    and generates velocity commands for the mobile base.

    Grid layout (indices 0-8):
        0: (-δ,+δ)   1: (0,+δ)   2: (+δ,+δ)
        3: (-δ, 0)   4: (0, 0)   5: (+δ, 0)    δ = grid spacing (default 0.10m)
        6: (-δ,-δ)   7: (0,-δ)   8: (+δ,-δ)

    Each RM is computed in the robot base frame at that grid position.
    The workspace region moves relative to each base position.

    The region is one or more spheres of `workspace_radius`, unioned: a single
    centre scores the ArTag alone, while goal + the tail of the MPC path scores
    the approach corridor the arm actually traverses. See compute_quality_scores.

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

    Velocity command:
        v = k · ∇Q
    """

    def __init__(self, workspace_radius=0.30, grid_spacing=0.10, gain=1.0,
                 max_linear_vel=0.10, device='cuda',
                 gradient_method='least_squares'):
        """
        Initialize gradient-based controller.

        Args:
            workspace_radius: float, radius of circular workspace in meters (default: 0.30m)
            grid_spacing: float, spacing δ between grid points in meters (default: 0.10m)
            gain: float, proportional gain k for velocity command (default: 1.0)
            max_linear_vel: float, maximum linear velocity in m/s (default: 0.10 m/s)
            device: torch device ('cuda' or 'cpu')
            gradient_method: 'least_squares' (default, uses all 9 maps) or 'central'
        """
        if gradient_method not in ('central', 'least_squares'):
            raise ValueError(
                f"gradient_method must be 'central' or 'least_squares', got {gradient_method!r}"
            )

        self.workspace_radius = workspace_radius
        self.delta = grid_spacing
        self.gain = gain
        self.max_linear_vel = max_linear_vel
        self.device = device
        self.gradient_method = gradient_method

        # Workspace region: one or more centers in the world frame. A single
        # center is the ArTag/goal alone; several describe the union of spheres
        # around the goal plus the tail of the MPC path (see compute_quality_scores).
        self.workspace_centers_world = None

        # Cached union mask for the untranslated (index 4) position, plus the key
        # it was built for. The region only moves when the goal/path moves, so a
        # static target rebuilds nothing between cycles.
        self._region_mask = None
        self._region_mask_key = None
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
        # same convention as _get_grid_offset and ObstacleMapTransformer.
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

        Args:
            centers_xyz: iterable of (x, y, z) tuples in meters
        """
        centers = [tuple(float(v) for v in c) for c in centers_xyz]
        if not centers:
            raise ValueError('Workspace region needs at least one center')
        self.workspace_centers_world = centers

    @property
    def workspace_center_world(self):
        """Primary center (goal). None until update_workspace_*() is called."""
        if self.workspace_centers_world is None:
            return None
        return self.workspace_centers_world[0]

    def _get_grid_offset(self, index):
        """
        Get the (x, y) offset for a given grid index.

        Args:
            index: int, grid index (0-8)

        Returns:
            tuple: (offset_x, offset_y) in meters
        """
        row = index // 3  # 0, 1, 2 (top to bottom)
        col = index % 3   # 0, 1, 2 (left to right)

        offset_x = (col - 1) * self.delta  # -δ, 0, +δ
        offset_y = (1 - row) * self.delta  # +δ, 0, -δ

        return offset_x, offset_y

    def _get_workspace_center_for_rm(self, rm_index):
        """
        Get the workspace center in the reference frame of a specific RM.

        When the base moves by (offset_x, offset_y), the workspace center
        moves by (-offset_x, -offset_y) in the base reference frame.

        Args:
            rm_index: int, index of the reachability map (0-8)

        Returns:
            tuple: (x, y, z) workspace center in RM reference frame
        """
        if self.workspace_center_world is None:
            raise RuntimeError("Workspace center not set. Call update_workspace_center() first.")

        tag_x, tag_y, tag_z = self.workspace_center_world
        offset_x, offset_y = self._get_grid_offset(rm_index)

        # Workspace center in this RM's reference frame
        center_x = tag_x - offset_x
        center_y = tag_y - offset_y
        center_z = tag_z

        return (center_x, center_y, center_z)

    def _get_region_centers_for_rm(self, rm_index):
        """
        Get every region center in the reference frame of a specific RM.

        Same displacement rule as _get_workspace_center_for_rm, applied to the
        whole region: when the base moves by (offset_x, offset_y), the world —
        goal and path alike — moves by (-offset_x, -offset_y) in the base frame.

        Args:
            rm_index: int, index of the reachability map (0-8)

        Returns:
            list: (x, y, z) centers in the RM reference frame
        """
        if self.workspace_centers_world is None:
            raise RuntimeError(
                "Workspace region not set. Call update_workspace_region() first.")

        offset_x, offset_y = self._get_grid_offset(rm_index)
        return [(cx - offset_x, cy - offset_y, cz)
                for cx, cy, cz in self.workspace_centers_world]

    def _build_region_mask(self, centers, vg_info):
        """Union-of-spheres mask over `centers`, at the workspace radius."""
        return VoxelMask.union_of_spheres(
            centers, self.workspace_radius,
            vg_info['origin'], vg_info['resolution'],
            vg_info['size_x'], vg_info['size_y'], vg_info['size_z'],
            device=self.device
        )

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
        if self.workspace_centers_world is None:
            raise RuntimeError(
                "Workspace region not set. Call update_workspace_region() first.")

        resolution = vg_info['resolution']
        shift_voxels = self.delta / resolution
        shift_int = int(round(shift_voxels))
        self.mask_shift_exact = abs(shift_voxels - shift_int) < 1e-6 and shift_int != 0

        base_mask = None
        if self.mask_shift_exact:
            origin = vg_info['origin']
            key = (tuple(self.workspace_centers_world), self.workspace_radius,
                   origin.x, origin.y, origin.z, resolution,
                   vg_info['size_x'], vg_info['size_y'], vg_info['size_z'])
            if self._region_mask is None or self._region_mask_key != key:
                self._region_mask = self._build_region_mask(
                    self.workspace_centers_world, vg_info)
                self._region_mask_key = key
            base_mask = self._region_mask

        scores = torch.zeros(9, dtype=reachability_maps.dtype, device=self.device)

        for i in range(9):
            if base_mask is not None:
                offset_x, offset_y = self._get_grid_offset(i)
                di = -int(round(offset_x / resolution))
                dj = -int(round(offset_y / resolution))
                mask = base_mask if (di == 0 and dj == 0) else base_mask.translated(di, dj)
            else:
                mask = self._build_region_mask(
                    self._get_region_centers_for_rm(i), vg_info)

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

    def gradient_to_velocity(self, grad_x, grad_y):
        """
        Convert gradient to velocity command with saturation.

        Args:
            grad_x, grad_y: float, gradient components

        Returns:
            tuple: (vx, vy) velocity command in m/s
        """
        # Proportional control: v = k · ∇Q
        vx = self.gain * grad_x
        vy = self.gain * grad_y

        # Saturate to maximum velocity
        vel_magnitude = np.sqrt(vx**2 + vy**2)
        if vel_magnitude > self.max_linear_vel:
            scale = self.max_linear_vel / vel_magnitude
            vx *= scale
            vy *= scale

        return vx, vy

    def create_twist_message(self, vx, vy):
        """
        Create ROS2 Twist message from velocity command.

        Args:
            vx, vy: float, velocity components in m/s

        Returns:
            geometry_msgs/Twist message
        """
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        return twist

    def compute_control(self, reachability_maps, vg_info):
        """
        Main control loop: compute gradient and generate velocity command.

        Args:
            reachability_maps: torch.Tensor, shape (9, size_x, size_y, size_z) on GPU
            vg_info: dict with voxel grid parameters

        Returns:
            twist_msg: geometry_msgs/Twist message
            debug_info: dict with gradient, scores, velocities, and timing for logging
        """
        t_start = time.time()

        # Compute quality scores for all 9 positions
        scores = self.compute_quality_scores(reachability_maps, vg_info)
        t_after_scores = time.time()

        # Compute gradient
        grad_x, grad_y = self.compute_gradient(scores)
        t_after_gradient = time.time()

        # Generate velocity command
        vx, vy = self.gradient_to_velocity(grad_x, grad_y)
        t_after_velocity = time.time()

        # Create Twist message
        twist_msg = self.create_twist_message(vx, vy)
        t_after_message = time.time()

        t_end = time.time()

        # Debug info for logging
        debug_info = {
            'scores': scores.cpu().numpy(),
            'gradient': (grad_x, grad_y),
            'velocity': (vx, vy),
            'gradient_magnitude': np.sqrt(grad_x**2 + grad_y**2),
            'velocity_magnitude': np.sqrt(vx**2 + vy**2),
            'score_center': scores[self.idx_center].item(),
            'gradient_method': self.gradient_method,
            'region_size': len(self.workspace_centers_world),
            'mask_shift_exact': self.mask_shift_exact,
            'timing': {
                'score_computation': (t_after_scores - t_start) * 1000,
                'gradient_computation': (t_after_gradient - t_after_scores) * 1000,
                'velocity_generation': (t_after_velocity - t_after_gradient) * 1000,
                'message_creation': (t_after_message - t_after_velocity) * 1000,
                'total': (t_end - t_start) * 1000
            }
        }

        return twist_msg, debug_info

    def get_grid_positions_world(self, base_position):
        """
        Get the 9 grid positions in world coordinates given current base position.

        Useful for visualization or RM computation planning.

        Args:
            base_position: tuple (x, y) current base position in meters

        Returns:
            list: 9 tuples (x, y) representing grid positions in world frame
        """
        base_x, base_y = base_position
        positions = []

        for i in range(9):
            offset_x, offset_y = self._get_grid_offset(i)
            positions.append((base_x + offset_x, base_y + offset_y))

        return positions
