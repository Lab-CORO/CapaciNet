#!/usr/bin/env python3

import torch
import numpy as np
import time
from geometry_msgs.msg import Twist
from .workspace_evaluation import WorkspaceEvaluation


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
    The workspace center (ArTag) moves relative to each base position.

    Gradient computation (central finite differences):
        ∂Q/∂x = (Q[5] - Q[3]) / (2δ)
        ∂Q/∂y = (Q[1] - Q[7]) / (2δ)

    Velocity command:
        v = k · ∇Q
    """

    def __init__(self, workspace_radius=0.30, grid_spacing=0.10, gain=1.0,
                 max_linear_vel=0.10, device='cuda'):
        """
        Initialize gradient-based controller.

        Args:
            workspace_radius: float, radius of circular workspace in meters (default: 0.30m)
            grid_spacing: float, spacing δ between grid points in meters (default: 0.10m)
            gain: float, proportional gain k for velocity command (default: 1.0)
            max_linear_vel: float, maximum linear velocity in m/s (default: 0.10 m/s)
            device: torch device ('cuda' or 'cpu')
        """
        self.workspace_radius = workspace_radius
        self.delta = grid_spacing
        self.gain = gain
        self.max_linear_vel = max_linear_vel
        self.device = device

        # Current workspace center (ArTag position in world frame)
        self.workspace_center_world = None

        # Indices for gradient computation (central differences)
        self.idx_center = 4
        self.idx_x_plus = 5   # (+δ, 0)
        self.idx_x_minus = 3  # (-δ, 0)
        self.idx_y_plus = 1   # (0, +δ)
        self.idx_y_minus = 7  # (0, -δ)

    def update_workspace_center(self, workspace_center_xyz):
        """
        Update the workspace center (ArTag position in world frame).

        Args:
            workspace_center_xyz: tuple (x, y, z) in meters
        """
        self.workspace_center_world = workspace_center_xyz

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

    def compute_quality_scores(self, reachability_maps, vg_info):
        """
        Compute quality scores Q for all 9 reachability maps.

        For each RM, the workspace center is adjusted based on the grid position.

        Args:
            reachability_maps: torch.Tensor, shape (9, size_x, size_y, size_z) on GPU
            vg_info: dict with keys 'origin', 'resolution', 'size_x', 'size_y', 'size_z'

        Returns:
            torch.Tensor: Quality scores, shape (9,), on GPU
        """
        scores = torch.zeros(9, dtype=reachability_maps.dtype, device=self.device)

        for i in range(9):
            # Get workspace center for this RM
            workspace_center = self._get_workspace_center_for_rm(i)

            # Create WorkspaceEvaluation for this specific center
            workspace_eval = WorkspaceEvaluation(
                center_xyz=workspace_center,
                radius=self.workspace_radius,
                device=self.device
            )

            # Compute quality score
            score = workspace_eval.compute_quality_score(
                reachability_maps[i],
                vg_info['origin'],
                vg_info['resolution'],
                vg_info['size_x'],
                vg_info['size_y'],
                vg_info['size_z']
            )
            scores[i] = score

        return scores

    def compute_gradient(self, scores):
        """
        Compute spatial gradient ∇Q using central finite differences.

        Args:
            scores: torch.Tensor, shape (9,), quality scores on GPU

        Returns:
            tuple: (grad_x, grad_y) gradient components (float)
        """
        # Central finite differences
        grad_x = (scores[self.idx_x_plus] - scores[self.idx_x_minus]) / (2 * self.delta)
        grad_y = (scores[self.idx_y_plus] - scores[self.idx_y_minus]) / (2 * self.delta)

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
