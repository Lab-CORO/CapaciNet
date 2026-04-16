#!/usr/bin/env python3

import torch
import numpy as np
import yaml


class VoxelMask:
    """
    Generic 3D voxel mask on GPU with boolean operations.

    Can be created from:
    - Existing mask tensor
    - Dictionary definition (YAML-compatible) with primitive shapes
    - YAML file path

    Supports primitive shapes: cuboid, sphere

    The mask is stored as a boolean tensor on GPU to minimize memory usage.
    """

    def __init__(self, mask_tensor=None, shape=None, device='cuda', fill_value=False):
        """
        Initialize voxel mask from existing tensor or empty shape.

        For creation from definitions, use VoxelMask.from_definition() instead.

        Args:
            mask_tensor: torch.Tensor, shape (size_x, size_y, size_z), bool or float
                        If None, creates empty mask with given shape
            shape: tuple (size_x, size_y, size_z), used if mask_tensor is None
            device: torch device ('cuda' or 'cpu')
            fill_value: bool, initial value if creating from shape (default: False)
        """
        self.device = device

        if mask_tensor is not None:
            # Convert to boolean tensor
            if mask_tensor.dtype == torch.bool:
                self.mask = mask_tensor.to(device)
            else:
                self.mask = (mask_tensor > 0.5).to(device)
        elif shape is not None:
            # Create empty mask
            self.mask = torch.full(shape, fill_value, dtype=torch.bool, device=device)
        else:
            raise ValueError("Either mask_tensor or shape must be provided")

    @property
    def shape(self):
        """Get mask shape (size_x, size_y, size_z)."""
        return self.mask.shape

    def clone(self):
        """Create a deep copy of this mask."""
        return VoxelMask(mask_tensor=self.mask.clone(), device=self.device)

    # ==================== Boolean Operations ====================

    def subtract(self, other_mask):
        """
        Subtract another mask: self AND NOT other.

        Args:
            other_mask: VoxelMask instance

        Returns:
            VoxelMask: New mask with subtraction result
        """
        result = self.mask & (~other_mask.mask)
        return VoxelMask(mask_tensor=result, device=self.device)

    def add(self, other_mask):
        """
        Add another mask: self OR other (union).

        Args:
            other_mask: VoxelMask instance

        Returns:
            VoxelMask: New mask with union result
        """
        result = self.mask | other_mask.mask
        return VoxelMask(mask_tensor=result, device=self.device)

    def intersect(self, other_mask):
        """
        Intersect with another mask: self AND other.

        Args:
            other_mask: VoxelMask instance

        Returns:
            VoxelMask: New mask with intersection result
        """
        result = self.mask & other_mask.mask
        return VoxelMask(mask_tensor=result, device=self.device)

    def invert(self):
        """
        Invert mask: NOT self.

        Returns:
            VoxelMask: New mask with inverted values
        """
        result = ~self.mask
        return VoxelMask(mask_tensor=result, device=self.device)

    # ==================== Application to Voxelmaps ====================

    def apply_to_voxelmap(self, voxelmap, masked_value=0.0):
        """
        Apply mask to a voxel map (set masked voxels to a value).

        Args:
            voxelmap: torch.Tensor on GPU
            masked_value: float, value to set for masked voxels (default: 0.0)

        Returns:
            torch.Tensor: Masked voxel map (same shape as input)
        """
        result = voxelmap.clone()
        result[self.mask] = masked_value
        return result

    def extract_from_voxelmap(self, voxelmap):
        """
        Extract voxel values where mask is True.

        Args:
            voxelmap: torch.Tensor on GPU

        Returns:
            torch.Tensor: Voxel values at masked locations (1D tensor)
        """
        return voxelmap[self.mask]

    def compute_sum(self, voxelmap):
        """
        Compute sum of voxel values where mask is True.

        Args:
            voxelmap: torch.Tensor on GPU

        Returns:
            torch.Tensor: Sum of masked values (scalar tensor)
        """
        return torch.sum(voxelmap[self.mask])

    def compute_mean(self, voxelmap):
        """
        Compute mean of voxel values where mask is True.

        Args:
            voxelmap: torch.Tensor on GPU

        Returns:
            float: Mean of masked values, or 0.0 if mask is empty
        """
        count = self.count()
        if count > 0:
            return (torch.sum(voxelmap[self.mask]) / count).item()
        else:
            return 0.0

    # ==================== Conversions ====================

    def to_float(self):
        """
        Convert to float tensor (0.0 or 1.0).

        Returns:
            torch.Tensor: Float representation of mask
        """
        return self.mask.float()

    def to_numpy(self):
        """
        Convert to numpy array (CPU).

        Returns:
            numpy.ndarray: Boolean array on CPU
        """
        return self.mask.cpu().numpy()

    # ==================== Utilities ====================

    def count(self):
        """
        Count number of True voxels.

        Returns:
            int: Number of voxels where mask is True
        """
        return torch.sum(self.mask).item()

    def is_empty(self):
        """Check if mask has no True voxels."""
        return self.count() == 0

    def get_bounds(self):
        """
        Get bounding box of True voxels in index space.

        Returns:
            dict: {'i_min', 'i_max', 'j_min', 'j_max', 'k_min', 'k_max'}
                  or None if mask is empty
        """
        if self.is_empty():
            return None

        indices = torch.nonzero(self.mask, as_tuple=False)  # (N, 3)
        i_coords = indices[:, 0]
        j_coords = indices[:, 1]
        k_coords = indices[:, 2]

        return {
            'i_min': int(torch.min(i_coords).item()),
            'i_max': int(torch.max(i_coords).item()),
            'j_min': int(torch.min(j_coords).item()),
            'j_max': int(torch.max(j_coords).item()),
            'k_min': int(torch.min(k_coords).item()),
            'k_max': int(torch.max(k_coords).item()),
        }

    # ==================== Operators ====================

    def __and__(self, other):
        """Operator: self & other."""
        return self.intersect(other)

    def __or__(self, other):
        """Operator: self | other."""
        return self.add(other)

    def __invert__(self):
        """Operator: ~self."""
        return self.invert()

    def __sub__(self, other):
        """Operator: self - other."""
        return self.subtract(other)

    # ==================== Factory Methods ====================

    @staticmethod
    def from_definition(definition, origin, resolution, size_x, size_y, size_z, device='cuda'):
        """
        Create mask from dictionary definition (YAML-compatible).

        Supports multiple primitive shapes that are combined with OR operation.

        Args:
            definition: dict with structure:
                {
                    'cuboid': {
                        'name1': {'dims': [x, y, z], 'pose': [x, y, z, qx, qy, qz, qw]},
                        'name2': {...}
                    },
                    'sphere': {
                        'name3': {'center': [x, y, z], 'radius': r},
                        'name4': {...}
                    }
                }
            origin: Point3D-like with .x, .y, .z (voxel grid origin)
            resolution: float, voxel size in meters
            size_x, size_y, size_z: int, voxel grid dimensions
            device: torch device

        Returns:
            VoxelMask: Combined mask from all shapes
        """
        # Start with empty mask
        combined_mask = VoxelMask(shape=(size_x, size_y, size_z), device=device, fill_value=False)

        # Process cuboids
        if 'cuboid' in definition:
            for name, params in definition['cuboid'].items():
                cuboid_mask = VoxelMask._create_cuboid(
                    params['dims'], params['pose'],
                    origin, resolution, size_x, size_y, size_z, device
                )
                combined_mask = combined_mask.add(cuboid_mask)

        # Process spheres
        if 'sphere' in definition:
            for name, params in definition['sphere'].items():
                sphere_mask = VoxelMask._create_sphere(
                    params['center'], params['radius'],
                    origin, resolution, size_x, size_y, size_z, device
                )
                combined_mask = combined_mask.add(sphere_mask)

        return combined_mask

    @staticmethod
    def from_yaml(yaml_path, origin, resolution, size_x, size_y, size_z, device='cuda'):
        """
        Create mask from YAML file.

        Args:
            yaml_path: str, path to YAML file
            origin, resolution, size_x, size_y, size_z: voxel grid parameters
            device: torch device

        Returns:
            VoxelMask: Mask from YAML definition
        """
        with open(yaml_path, 'r') as f:
            definition = yaml.safe_load(f)
        return VoxelMask.from_definition(definition, origin, resolution,
                                          size_x, size_y, size_z, device)

    @staticmethod
    def _create_cuboid(dims, pose, origin, resolution, size_x, size_y, size_z, device):
        """
        Create mask from an axis-aligned cuboid.

        Args:
            dims: list [dx, dy, dz], dimensions in meters
            pose: list [x, y, z, qx, qy, qz, qw], pose in world frame
                  (rotation currently ignored, assumes axis-aligned)
            origin, resolution, size_x, size_y, size_z: voxel grid parameters
            device: torch device

        Returns:
            VoxelMask: Mask representing the cuboid
        """
        # Parse pose
        cx, cy, cz = pose[0], pose[1], pose[2]
        dx, dy, dz = dims[0], dims[1], dims[2]

        # Calculate cuboid bounds in world coordinates
        x_min = cx - dx / 2.0
        x_max = cx + dx / 2.0
        y_min = cy - dy / 2.0
        y_max = cy + dy / 2.0
        z_min = cz - dz / 2.0
        z_max = cz + dz / 2.0

        # Convert to voxel indices
        i_min = int(np.floor((x_min - origin.x) / resolution))
        i_max = int(np.ceil((x_max - origin.x) / resolution))
        j_min = int(np.floor((y_min - origin.y) / resolution))
        j_max = int(np.ceil((y_max - origin.y) / resolution))
        k_min = int(np.floor((z_min - origin.z) / resolution))
        k_max = int(np.ceil((z_max - origin.z) / resolution))

        # Clamp to grid bounds
        i_min = max(0, min(i_min, size_x))
        i_max = max(0, min(i_max, size_x))
        j_min = max(0, min(j_min, size_y))
        j_max = max(0, min(j_max, size_y))
        k_min = max(0, min(k_min, size_z))
        k_max = max(0, min(k_max, size_z))

        # Create mask
        mask = torch.zeros((size_x, size_y, size_z), dtype=torch.bool, device=device)
        if i_max > i_min and j_max > j_min and k_max > k_min:
            mask[i_min:i_max, j_min:j_max, k_min:k_max] = True

        return VoxelMask(mask_tensor=mask, device=device)

    @staticmethod
    def _create_sphere(center, radius, origin, resolution, size_x, size_y, size_z, device):
        """
        Create mask from a spherical region.

        Args:
            center: list [x, y, z] in meters
            radius: float, radius in meters
            origin, resolution, size_x, size_y, size_z: voxel grid parameters
            device: torch device

        Returns:
            VoxelMask: Mask representing the sphere
        """
        cx, cy, cz = center[0], center[1], center[2]

        # Convert center to voxel coordinates
        center_i = (cx - origin.x) / resolution
        center_j = (cy - origin.y) / resolution
        center_k = (cz - origin.z) / resolution

        # Create coordinate grids
        i_coords = torch.arange(size_x, dtype=torch.float32, device=device)
        j_coords = torch.arange(size_y, dtype=torch.float32, device=device)
        k_coords = torch.arange(size_z, dtype=torch.float32, device=device)

        # Create 3D meshgrid
        i_grid, j_grid, k_grid = torch.meshgrid(i_coords, j_coords, k_coords, indexing='ij')

        # Calculate distance from center
        dist_voxels = torch.sqrt(
            (i_grid - center_i)**2 +
            (j_grid - center_j)**2 +
            (k_grid - center_k)**2
        )

        # Convert radius to voxel units
        radius_voxels = radius / resolution

        # Create spherical mask
        mask = dist_voxels <= radius_voxels

        return VoxelMask(mask_tensor=mask, device=device)

    @staticmethod
    def circular_2d(center_xy, radius, origin, resolution, size_x, size_y, size_z, device='cuda'):
        """
        Create cylindrical mask (circle in XY plane, unbounded in Z).

        Useful for workspace definition.

        Args:
            center_xy: tuple (x, y) in meters
            radius: float, radius in meters
            origin, resolution, size_x, size_y, size_z: voxel grid parameters
            device: torch device

        Returns:
            VoxelMask: Cylindrical mask
        """
        cx, cy = center_xy

        # Convert center to voxel coordinates
        center_i = (cx - origin.x) / resolution
        center_j = (cy - origin.y) / resolution

        # Create coordinate grids for X and Y
        i_coords = torch.arange(size_x, dtype=torch.float32, device=device)
        j_coords = torch.arange(size_y, dtype=torch.float32, device=device)

        # Create 2D meshgrid
        i_grid, j_grid = torch.meshgrid(i_coords, j_coords, indexing='ij')

        # Calculate 2D distance from center
        dist_voxels = torch.sqrt((i_grid - center_i)**2 + (j_grid - center_j)**2)

        # Convert radius to voxel units
        radius_voxels = radius / resolution

        # Create 2D circular mask
        mask_2d = dist_voxels <= radius_voxels

        # Expand to 3D (cylinder)
        mask_3d = mask_2d.unsqueeze(2).expand(size_x, size_y, size_z)

        return VoxelMask(mask_tensor=mask_3d, device=device)
