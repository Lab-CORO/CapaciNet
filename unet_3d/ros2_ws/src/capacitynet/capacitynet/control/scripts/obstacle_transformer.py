#!/usr/bin/env python3

import torch
import yaml
from .voxel_mask import VoxelMask, translate_filled


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
            raise ValueError('Expected 5D tensor (batch, channels, size_x, size_y, '
                             f'size_z), got shape {voxel_map.shape}')

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

        # Obstacles move opposite to the base: stepping the base to
        # (+delta_x, +delta_y) makes the world appear to move by
        # (-delta_x, -delta_y) in the base frame.
        #
        # This must stay character-for-character the shift that
        # GradientBasedController.compute_quality_scores applies to the region
        # mask — the mask scores the map produced here, so if the two disagree
        # on sign or axis, Q is read somewhere the obstacles are not. Both go
        # through translate_filled for exactly that reason.
        di = -int(round(delta_x / self.resolution))
        dj = -int(round(delta_y / self.resolution))

        # voxel_map is (batch, channels, size_x, size_y, size_z): x and y are
        # axes 2 and 3. Cells with no source after the shift are filled with -1
        # (unknown/unobserved, distinct from 0=free and 1=occupied), and nothing
        # wraps around from the opposite face.
        transformed = translate_filled(voxel_map, (0, 0, di, dj), fill_value=-1)

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

    def generate_grid_transforms(self, voxel_map, grid_spacing=0.10, grid_size=3, origin=None):
        """
        Generate grid_size**2 transformed voxel maps for a grid_size x grid_size grid.

        Grid layout for grid_size=3 (the default):
            0: (-δ,+δ)   1: (0,+δ)   2: (+δ,+δ)
            3: (-δ, 0)   4: (0, 0)   5: (+δ, 0)
            6: (-δ,-δ)   7: (0,-δ)   8: (+δ,-δ)
        Larger odd grid_size extends the same lattice outward in steps of δ,
        row-major (top to bottom, left to right) — same order as
        GradientBasedController.grid_offsets().

        Args:
            voxel_map: torch.Tensor, shape (1, 1, size_x, size_y, size_z) on GPU
            grid_spacing: float, spacing δ in meters (default: 0.10m)
            grid_size: int, odd, >= 3 (default: 3)
            origin: Point3D-like with .x, .y, .z (required if static obstacles defined)

        Returns:
            torch.Tensor: shape (grid_size**2, 1, size_x, size_y, size_z)
        """
        if grid_size < 3 or grid_size % 2 == 0:
            raise ValueError(f'grid_size must be odd and >= 3, got {grid_size!r}')

        half = grid_size // 2
        transformed_maps = []

        for row in range(grid_size):  # 0..grid_size-1 (top to bottom)
            for col in range(grid_size):  # 0..grid_size-1 (left to right)
                # Calculate offset for this grid position
                offset_x = (col - half) * grid_spacing
                offset_y = (half - row) * grid_spacing

                # Apply transformation
                transformed = self.transform(voxel_map, offset_x, offset_y, origin=origin)
                transformed_maps.append(transformed)

        # Stack into single tensor
        return torch.cat(transformed_maps, dim=0)  # (grid_size**2, 1, size_x, size_y, size_z)

    def update_resolution(self, new_resolution):
        """
        Update voxel resolution (e.g., if scene resolution changes).

        Args:
            new_resolution: float, new voxel size in meters
        """
        self.resolution = new_resolution
