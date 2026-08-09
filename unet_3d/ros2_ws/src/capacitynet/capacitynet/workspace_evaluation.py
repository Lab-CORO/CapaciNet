#!/usr/bin/env python3

import torch

from .voxel_mask import VoxelMask

_coord_grid_cache = {}


def _get_coord_grids(size_x, size_y, size_z, device):
    """Return a cached torch.meshgrid of voxel indices, shared across sphere centers.

    Rebuilding a (size_x, size_y, size_z) meshgrid per pose is the dominant
    cost when scoring many centers (e.g. an MPC path) against the same grid.
    """
    key = (size_x, size_y, size_z, device)
    grids = _coord_grid_cache.get(key)
    if grids is None:
        i = torch.arange(size_x, dtype=torch.float32, device=device)
        j = torch.arange(size_y, dtype=torch.float32, device=device)
        k = torch.arange(size_z, dtype=torch.float32, device=device)
        grids = torch.meshgrid(i, j, k, indexing='ij')
        _coord_grid_cache[key] = grids
    return grids


class WorkspaceEvaluation:
    """
    Evaluate reachability quality score within a spherical workspace region.

    The workspace is a 3D sphere (ball) centered at center_xyz with the given
    radius; center_z is meaningful (unlike the previous cylindrical definition).

    All operations stay on GPU to avoid CPU-GPU transfers.
    """

    def __init__(self, center_xyz, radius=0.30, device='cuda'):
        """
        Args:
            center_xyz: tuple (x, y, z) in meters for the spherical workspace center
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

        mask = VoxelMask.sphere_3d(
            center_xyz=self._center_xyz,
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

    @staticmethod
    def compute_quality_scores_batched(
            reachability_map, centers, origin, resolution,
            size_x, size_y, size_z, radius, device='cuda'):
        """
        Compute mean reachability inside a sphere of `radius` around each center.

        Same region definition as VoxelMask.sphere_3d (squared distance in voxel
        units vs radius/resolution), but the voxel coordinate grids are built
        once and reused for every center instead of once per center.

        Args:
            reachability_map: torch.Tensor on GPU, shape (size_x, size_y, size_z)
            centers: iterable of (x, y, z) tuples in meters
            origin, resolution, size_x, size_y, size_z: voxel grid parameters
            radius: float, sphere radius in meters
            device: torch device ('cuda' or 'cpu')

        Returns:
            list[float]: mean reachability for each center, 0.0 for an empty sphere
        """
        i_grid, j_grid, k_grid = _get_coord_grids(size_x, size_y, size_z, device)
        radius_voxels_sq = (radius / resolution) ** 2

        scores = []
        for cx, cy, cz in centers:
            center_i = (cx - origin.x) / resolution
            center_j = (cy - origin.y) / resolution
            center_k = (cz - origin.z) / resolution

            dist_sq = ((i_grid - center_i) ** 2 + (j_grid - center_j) ** 2
                       + (k_grid - center_k) ** 2)
            mask = dist_sq <= radius_voxels_sq

            count = torch.sum(mask)
            if count.item() > 0:
                score = (torch.sum(reachability_map[mask]) / count).item()
            else:
                score = 0.0
            scores.append(score)

        return scores
