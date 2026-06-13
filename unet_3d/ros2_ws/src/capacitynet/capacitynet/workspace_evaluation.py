#!/usr/bin/env python3

from .voxel_mask import VoxelMask


class WorkspaceEvaluation:
    """
    Evaluate reachability quality score within a circular workspace region.

    All operations stay on GPU to avoid CPU-GPU transfers.
    """

    def __init__(self, center_xyz, radius=0.30, device='cuda'):
        """
        Args:
            center_xyz: tuple (x, y, z) in meters for circular workspace center
            radius: float, radius in meters (default: 0.30m)
            device: torch device ('cuda' or 'cpu')
        """
        self.device = device
        self._center_xyz = center_xyz
        self._radius = radius

        self._cached_mask = None
        self._cache_key = None

    def _generate_mask(self, origin, resolution, size_x, size_y, size_z):
        cache_key = (origin.x, origin.y, origin.z, resolution, size_x, size_y, size_z)
        if self._cached_mask is not None and self._cache_key == cache_key:
            return self._cached_mask

        mask = VoxelMask.circular_2d(
            center_xy=(self._center_xyz[0], self._center_xyz[1]),
            radius=self._radius,
            origin=origin,
            resolution=resolution,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            device=self.device
        )

        self._cached_mask = mask
        self._cache_key = cache_key
        return mask

    def compute_quality_score(self, reachability_map, origin, resolution, size_x, size_y, size_z):
        """
        Compute quality score Q: mean reachability within the workspace.

        Args:
            reachability_map: torch.Tensor on GPU, shape (size_x, size_y, size_z)
            origin, resolution, size_x, size_y, size_z: voxel grid parameters

        Returns:
            float: Mean reachability value inside workspace
        """
        mask = self._generate_mask(origin, resolution, size_x, size_y, size_z)
        return mask.compute_mean(reachability_map)
