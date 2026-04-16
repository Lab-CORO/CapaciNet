#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import torch
import numpy as np
import time
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Header

from .obstacle_transformer import ObstacleMapTransformer
from .gradient_controller import GradientBasedController
from .voxel_mask import VoxelMask


class GradientControllerMockNode(Node):
    """
    Mock node for testing the gradient-based controller pipeline.

    This node:
    1. Creates synthetic voxel maps and reachability maps
    2. Simulates the 9-grid transformation
    3. Computes gradient and velocity commands
    4. Publishes cmd_vel commands
    5. Logs debug information
    """

    def __init__(self):
        super().__init__('gradient_controller_mock')

        # Parameters
        self.declare_parameter('grid_spacing', 0.10)
        self.declare_parameter('control_frequency', 1.0)  # Hz
        self.declare_parameter('gain', 1.0)
        self.declare_parameter('workspace_radius', 0.30)
        self.declare_parameter('voxel_resolution', 0.02)
        self.declare_parameter('use_static_obstacles', False)
        self.declare_parameter('static_obstacles_yaml', '')

        # Get parameters
        grid_spacing = self.get_parameter('grid_spacing').value
        control_freq = self.get_parameter('control_frequency').value
        gain = self.get_parameter('gain').value
        workspace_radius = self.get_parameter('workspace_radius').value
        self.voxel_resolution = self.get_parameter('voxel_resolution').value
        use_static_obstacles = self.get_parameter('use_static_obstacles').value
        static_yaml = self.get_parameter('static_obstacles_yaml').value

        # Device
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device: {self.device}")

        # Initialize obstacle transformer
        if use_static_obstacles and static_yaml:
            self.obstacle_transformer = ObstacleMapTransformer(
                resolution=self.voxel_resolution,
                device=self.device,
                static_obstacles_yaml=static_yaml
            )
            self.get_logger().info(f"Loaded static obstacles from: {static_yaml}")
        else:
            self.obstacle_transformer = ObstacleMapTransformer(
                resolution=self.voxel_resolution,
                device=self.device
            )
            self.get_logger().info("No static obstacles loaded")

        # Initialize gradient controller
        self.gradient_controller = GradientBasedController(
            workspace_radius=workspace_radius,
            grid_spacing=grid_spacing,
            gain=gain,
            max_linear_vel=0.10,
            device=self.device
        )
        self.get_logger().info(
            f"Controller initialized: radius={workspace_radius}m, "
            f"spacing={grid_spacing}m, gain={gain}"
        )

        # Set workspace center (simulated ArTag position)
        self.workspace_center = (1.0, 0.5, 0.8)  # x, y, z in meters
        self.gradient_controller.update_workspace_center(self.workspace_center)
        self.get_logger().info(f"Workspace center: {self.workspace_center}")

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create mock voxel grid info
        self.vg_info = self._create_mock_voxel_grid_info()

        # Timer for control loop
        timer_period = 1.0 / control_freq
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.iteration = 0
        self.get_logger().info(f"Mock node started at {control_freq} Hz")

    def _create_mock_voxel_grid_info(self):
        """Create mock voxel grid parameters."""
        # Create a simple Point-like object for origin
        class MockPoint:
            def __init__(self, x, y, z):
                self.x = x
                self.y = y
                self.z = z

        return {
            'origin': MockPoint(0.0, 0.0, -0.5),
            'resolution': self.voxel_resolution,
            'size_x': 100,
            'size_y': 100,
            'size_z': 50
        }

    def _create_mock_voxel_map(self):
        """
        Create a synthetic voxel map for testing.

        Returns a simple obstacle map with a few random obstacles.
        """
        size_x = self.vg_info['size_x']
        size_y = self.vg_info['size_y']
        size_z = self.vg_info['size_z']

        # Create empty voxel map
        voxel_map = torch.zeros((1, 1, size_x, size_y, size_z),
                                 dtype=torch.float32, device=self.device)

        # Add some random obstacles (10% occupancy)
        num_obstacles = int(0.1 * size_x * size_y * size_z)
        i_coords = torch.randint(0, size_x, (num_obstacles,), device=self.device)
        j_coords = torch.randint(0, size_y, (num_obstacles,), device=self.device)
        k_coords = torch.randint(0, size_z, (num_obstacles,), device=self.device)

        voxel_map[0, 0, i_coords, j_coords, k_coords] = 1.0

        return voxel_map

    def _simulate_reachability_prediction(self, transformed_voxels):
        """
        Simulate reachability map predictions.

        In reality, this would be done by the UNet3D model.
        Here we create synthetic reachability maps with a gradient
        pointing towards the workspace center.

        Simulates 60ms inference time (realistic for GPU UNet3D).

        Args:
            transformed_voxels: torch.Tensor (9, 1, size_x, size_y, size_z)

        Returns:
            torch.Tensor: (9, size_x, size_y, size_z) simulated reachability maps
        """
        # Simulate UNet3D inference time (60ms for 9 RMs in batch)
        time.sleep(0.060)

        batch_size = transformed_voxels.shape[0]
        size_x = self.vg_info['size_x']
        size_y = self.vg_info['size_y']
        size_z = self.vg_info['size_z']

        # Create synthetic reachability maps
        # Higher reachability near the workspace center
        predictions = []

        for i in range(batch_size):
            # Create coordinates
            i_coords = torch.arange(size_x, dtype=torch.float32, device=self.device)
            j_coords = torch.arange(size_y, dtype=torch.float32, device=self.device)
            k_coords = torch.arange(size_z, dtype=torch.float32, device=self.device)

            i_grid, j_grid, k_grid = torch.meshgrid(i_coords, j_coords, k_coords, indexing='ij')

            # Convert workspace center to voxel coordinates
            center_i = (self.workspace_center[0] - self.vg_info['origin'].x) / self.vg_info['resolution']
            center_j = (self.workspace_center[1] - self.vg_info['origin'].y) / self.vg_info['resolution']
            center_k = (self.workspace_center[2] - self.vg_info['origin'].z) / self.vg_info['resolution']

            # Calculate distance from workspace center
            dist = torch.sqrt(
                (i_grid - center_i)**2 +
                (j_grid - center_j)**2 +
                (k_grid - center_k)**2
            )

            # Reachability decreases with distance (Gaussian-like)
            sigma = 30.0  # voxels
            reachability = torch.exp(-dist**2 / (2 * sigma**2))

            # Reduce reachability where there are obstacles
            obstacles = transformed_voxels[i, 0]
            reachability = reachability * (1.0 - obstacles * 0.5)

            predictions.append(reachability)

        return torch.stack(predictions, dim=0)  # (9, size_x, size_y, size_z)

    def control_loop(self):
        """Main control loop executed at each timer tick."""
        t_start = time.time()

        self.iteration += 1
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Iteration {self.iteration}")
        self.get_logger().info(f"{'='*60}")

        # Step 1: Create mock voxel map
        self.get_logger().info("Creating mock voxel map...")
        voxel_map = self._create_mock_voxel_map()
        t_after_voxel_creation = time.time()

        # Step 2: Generate 9 transformed voxel maps
        self.get_logger().info("Generating 9 grid transformations...")
        transformed_voxels = self.obstacle_transformer.generate_grid_transforms(
            voxel_map,
            grid_spacing=self.gradient_controller.delta,
            origin=self.vg_info['origin']
        )
        if self.device == 'cuda':
            torch.cuda.synchronize()
        t_after_transforms = time.time()
        self.get_logger().info(f"Transformed voxels shape: {transformed_voxels.shape}")

        # Step 3: Simulate reachability map predictions (includes 60ms sleep)
        self.get_logger().info("Simulating reachability predictions (60ms)...")
        predictions = self._simulate_reachability_prediction(transformed_voxels)
        if self.device == 'cuda':
            torch.cuda.synchronize()
        t_after_predictions = time.time()
        self.get_logger().info(f"Predictions shape: {predictions.shape}")

        # Step 4: Compute gradient and velocity command
        self.get_logger().info("Computing control command...")
        twist_msg, debug_info = self.gradient_controller.compute_control(
            predictions,
            self.vg_info
        )
        t_after_control = time.time()

        # Step 5: Log debug information
        self.get_logger().info(f"\nQuality Scores (9 positions):")
        scores = debug_info['scores']
        self.get_logger().info(f"  Grid layout:")
        self.get_logger().info(f"    {scores[0]:.3f}  {scores[1]:.3f}  {scores[2]:.3f}")
        self.get_logger().info(f"    {scores[3]:.3f}  {scores[4]:.3f}  {scores[5]:.3f}")
        self.get_logger().info(f"    {scores[6]:.3f}  {scores[7]:.3f}  {scores[8]:.3f}")

        grad_x, grad_y = debug_info['gradient']
        self.get_logger().info(f"\nGradient:")
        self.get_logger().info(f"  ∇Q = ({grad_x:+.4f}, {grad_y:+.4f})")
        self.get_logger().info(f"  |∇Q| = {debug_info['gradient_magnitude']:.4f}")

        vx, vy = debug_info['velocity']
        self.get_logger().info(f"\nVelocity Command:")
        self.get_logger().info(f"  v = ({vx:+.4f}, {vy:+.4f}) m/s")
        self.get_logger().info(f"  |v| = {debug_info['velocity_magnitude']:.4f} m/s")

        self.get_logger().info(f"\nCenter Score: {debug_info['score_center']:.3f}")

        # Step 6: Publish velocity command
        self.cmd_vel_pub.publish(twist_msg)
        t_after_publish = time.time()

        t_end = time.time()

        # Log timing breakdown
        timing = debug_info['timing']
        self.get_logger().info(f"\n{'-'*60}")
        self.get_logger().info(f"Timing Breakdown [ms]:")
        self.get_logger().info(
            f"  Voxel Creation:     {(t_after_voxel_creation - t_start)*1000:6.1f} ms"
        )
        self.get_logger().info(
            f"  Grid Transforms:    {(t_after_transforms - t_after_voxel_creation)*1000:6.1f} ms"
        )
        self.get_logger().info(
            f"  RM Prediction:      {(t_after_predictions - t_after_transforms)*1000:6.1f} ms  (simulated: 60ms)"
        )
        self.get_logger().info(
            f"  Control Compute:    {(t_after_control - t_after_predictions)*1000:6.1f} ms"
        )
        self.get_logger().info(
            f"    ├─ Scores (9x):   {timing['score_computation']:6.1f} ms"
        )
        self.get_logger().info(
            f"    ├─ Gradient:      {timing['gradient_computation']:6.1f} ms"
        )
        self.get_logger().info(
            f"    ├─ Velocity:      {timing['velocity_generation']:6.1f} ms"
        )
        self.get_logger().info(
            f"    └─ Message:       {timing['message_creation']:6.1f} ms"
        )
        self.get_logger().info(
            f"  Publish:            {(t_after_publish - t_after_control)*1000:6.1f} ms"
        )
        self.get_logger().info(f"  {'─'*40}")
        self.get_logger().info(
            f"  TOTAL:              {(t_end - t_start)*1000:6.1f} ms"
        )
        self.get_logger().info(f"{'-'*60}\n")


def main(args=None):
    rclpy.init(args=args)
    node = GradientControllerMockNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
