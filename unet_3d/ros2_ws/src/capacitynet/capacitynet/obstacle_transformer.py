#!/usr/bin/env python3

import torch
import torch.nn.functional as F
import yaml
from .voxel_mask import VoxelMask


class ObstacleMapTransformer:
    """
    Transform obstacle voxel maps by applying 2D translations on GPU.

    Simulates robot base displacement by translating the obstacle map
    in the opposite direction. All operations stay on GPU to avoid
    CPU-GPU transfers.

    When the base moves by (δx, δy), obstacles move by (-δx, -δy)
    in the base reference frame.
    """

    def __init__(self, resolution, device='cuda', static_obstacles_yaml=None,
                 static_obstacles_definition=None):
        """
        Initialize obstacle map transformer.

        Args:
            resolution: float, voxel size in meters
            device: torch device ('cuda' or 'cpu')
            static_obstacles_yaml: str, path to YAML file with static obstacles (optional)
            static_obstacles_definition: dict, definition of static obstacles (optional)
        """
        self.resolution = resolution
        self.device = device

        # Static obstacles configuration
        self.static_obstacles_yaml = static_obstacles_yaml
        self.static_obstacles_definition = static_obstacles_definition

        # Cache for static obstacle mask
        self._static_mask = None
        self._static_mask_cache_key = None

    def _get_static_mask(self, origin, size_x, size_y, size_z):
        """
        Get or create static obstacle mask.

        Args:
            origin: Point3D-like with .x, .y, .z
            size_x, size_y, size_z: int, voxel grid dimensions

        Returns:
            VoxelMask or None: Static obstacle mask
        """
        # No static obstacles defined
        if self.static_obstacles_yaml is None and self.static_obstacles_definition is None:
            return None

        # Check cache
        cache_key = (origin.x, origin.y, origin.z, self.resolution, size_x, size_y, size_z)
        if self._static_mask is not None and self._static_mask_cache_key == cache_key:
            return self._static_mask

        # Load definition from YAML if provided
        if self.static_obstacles_yaml is not None:
            with open(self.static_obstacles_yaml, 'r') as f:
                definition = yaml.safe_load(f)
        else:
            definition = self.static_obstacles_definition

        # Create mask from definition
        static_mask = VoxelMask.from_definition(
            definition,
            origin, self.resolution,
            size_x, size_y, size_z,
            self.device
        )

        # Cache result
        self._static_mask = static_mask
        self._static_mask_cache_key = cache_key

        return static_mask

    def transform(self, voxel_map, delta_x, delta_y, origin=None):
        """
        Apply translation to voxel map, preserving static obstacles.

        Args:
            voxel_map: torch.Tensor, shape (1, 1, size_x, size_y, size_z) on GPU
            delta_x, delta_y: float, translation in meters (base displacement)
            origin: Point3D-like with .x, .y, .z (required if static obstacles defined)

        Returns:
            torch.Tensor: Transformed voxel map, same shape, on GPU
        """
        # Ensure correct shape
        if voxel_map.dim() != 5:
            raise ValueError(f"Expected 5D tensor (B, C, D, H, W), got shape {voxel_map.shape}")

        batch, channels, size_x, size_y, size_z = voxel_map.shape

        # Get static obstacle mask
        static_mask = None
        if origin is not None:
            static_mask = self._get_static_mask(origin, size_x, size_y, size_z)

        # Save static obstacle voxels if mask exists
        static_voxels = None
        if static_mask is not None:
            # Extract voxel map from 5D to 3D for masking
            voxel_map_3d = voxel_map.squeeze(0).squeeze(0)  # (size_x, size_y, size_z)
            static_voxels = static_mask.extract_from_voxelmap(voxel_map_3d)

            # Set static obstacles to 0 (free space) before transformation
            voxel_map_3d = static_mask.apply_to_voxelmap(voxel_map_3d, masked_value=0.0)
            voxel_map = voxel_map_3d.unsqueeze(0).unsqueeze(0)  # Back to 5D

        # Convert meters to voxel indices
        tx_voxels = delta_x / self.resolution
        ty_voxels = delta_y / self.resolution

        # Obstacles move in opposite direction of base
        tx_voxels = -tx_voxels
        ty_voxels = -ty_voxels

        # Convert to normalized coordinates [-1, 1] for affine_grid
        # PyTorch convention: (D, H, W) = (X, Y, Z)
        tx_norm = 2.0 * tx_voxels / size_x
        ty_norm = 2.0 * ty_voxels / size_y
        tz_norm = 0.0  # No translation in Z

        # Create 3D affine transformation matrix (3x4)
        # Identity rotation + translation
        theta = torch.tensor([
            [1, 0, 0, tx_norm],
            [0, 1, 0, ty_norm],
            [0, 0, 1, tz_norm]
        ], dtype=voxel_map.dtype, device=self.device).unsqueeze(0)  # (1, 3, 4)

        # Generate sampling grid
        grid = F.affine_grid(
            theta,
            voxel_map.size(),
            align_corners=False
        )

        # Sample voxel map at transformed positions
        # mode='nearest' for binary occupancy data
        # padding_mode='zeros' fills out-of-bounds with 0 (free space)
        transformed = F.grid_sample(
            voxel_map,
            grid,
            mode='nearest',
            padding_mode='zeros',
            align_corners=False
        )

        # Restore static obstacles at their original positions
        if static_mask is not None and static_voxels is not None:
            transformed_3d = transformed.squeeze(0).squeeze(0)  # (size_x, size_y, size_z)

            # Create a tensor with static obstacle values at mask positions
            # Start with current transformed map
            result = transformed_3d.clone()

            # Set static obstacle voxels back to their original values
            result[static_mask.mask] = static_voxels

            transformed = result.unsqueeze(0).unsqueeze(0)  # Back to 5D

        return transformed

    def generate_grid_transforms(self, voxel_map, grid_spacing=0.10, origin=None):
        """
        Generate 9 transformed voxel maps for a 3x3 grid.

        Grid layout:
            0: (-δ,+δ)   1: (0,+δ)   2: (+δ,+δ)
            3: (-δ, 0)   4: (0, 0)   5: (+δ, 0)
            6: (-δ,-δ)   7: (0,-δ)   8: (+δ,-δ)

        Args:
            voxel_map: torch.Tensor, shape (1, 1, size_x, size_y, size_z) on GPU
            grid_spacing: float, spacing δ in meters (default: 0.10m)
            origin: Point3D-like with .x, .y, .z (required if static obstacles defined)

        Returns:
            torch.Tensor: shape (9, 1, size_x, size_y, size_z), 9 transformed maps
        """
        transformed_maps = []

        # Generate 9 transformations
        for row in range(3):  # 0, 1, 2 (top to bottom)
            for col in range(3):  # 0, 1, 2 (left to right)
                # Calculate offset for this grid position
                offset_x = (col - 1) * grid_spacing  # -δ, 0, +δ
                offset_y = (1 - row) * grid_spacing  # +δ, 0, -δ

                # Apply transformation
                transformed = self.transform(voxel_map, offset_x, offset_y, origin=origin)
                transformed_maps.append(transformed)

        # Stack into single tensor
        return torch.cat(transformed_maps, dim=0)  # (9, 1, size_x, size_y, size_z)

    def batch_transform(self, voxel_map, deltas, origin=None):
        """
        Apply multiple transformations in batch.

        Args:
            voxel_map: torch.Tensor, shape (1, 1, size_x, size_y, size_z) on GPU
            deltas: list of tuples [(dx1, dy1), (dx2, dy2), ...] in meters
            origin: Point3D-like with .x, .y, .z (required if static obstacles defined)

        Returns:
            torch.Tensor: shape (N, 1, size_x, size_y, size_z), N transformed maps
        """
        transformed_maps = []

        for delta_x, delta_y in deltas:
            transformed = self.transform(voxel_map, delta_x, delta_y, origin=origin)
            transformed_maps.append(transformed)

        return torch.cat(transformed_maps, dim=0)

    def update_resolution(self, new_resolution):
        """
        Update voxel resolution (e.g., if scene resolution changes).

        Args:
            new_resolution: float, new voxel size in meters
        """
        self.resolution = new_resolution

    @staticmethod
    def prepare_voxel_map(voxel_array, device='cuda'):
        """
        Convert numpy array or 3D tensor to 5D tensor format.

        Args:
            voxel_array: numpy array or torch.Tensor, shape (size_x, size_y, size_z)
            device: torch device

        Returns:
            torch.Tensor: shape (1, 1, size_x, size_y, size_z) on GPU
        """
        if not isinstance(voxel_array, torch.Tensor):
            voxel_tensor = torch.from_numpy(voxel_array)
        else:
            voxel_tensor = voxel_array

        # Add batch and channel dimensions
        if voxel_tensor.dim() == 3:
            voxel_tensor = voxel_tensor.unsqueeze(0).unsqueeze(0)

        return voxel_tensor.to(device)

    @staticmethod
    def get_grid_offsets(grid_spacing=0.10):
        """
        Get the 9 grid offsets as a list of (dx, dy) tuples.

        Args:
            grid_spacing: float, spacing δ in meters

        Returns:
            list: 9 tuples (dx, dy) in meters
        """
        offsets = []
        for row in range(3):
            for col in range(3):
                offset_x = (col - 1) * grid_spacing
                offset_y = (1 - row) * grid_spacing
                offsets.append((offset_x, offset_y))
        return offsets
