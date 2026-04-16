#!/usr/bin/env python3

from .voxel_mask import VoxelMask


class WorkspaceEvaluation:
    """
    Evaluate reachability quality score within a defined workspace.

    The workspace can be defined as:
    - Simple circular region (cylinder in XY plane)
    - Complex shape from dictionary definition (YAML-compatible)
    - Custom VoxelMask instance

    All operations stay on GPU to avoid CPU-GPU transfers.
    """

    def __init__(self, workspace_mask=None, center_xyz=None, radius=0.30,
                 workspace_definition=None, device='cuda'):
        """
        Initialize workspace evaluation.

        Priority (if multiple provided):
        1. workspace_mask (VoxelMask instance)
        2. workspace_definition (dict)
        3. center_xyz + radius (simple circular workspace)

        Args:
            workspace_mask: VoxelMask instance (directly provided)
            center_xyz: tuple (x, y, z) in meters for circular workspace
            radius: float, radius for circular workspace in meters (default: 0.30m)
            workspace_definition: dict for complex workspace (YAML-compatible), e.g.:
                {
                    'sphere': {
                        'workspace': {'center': [1.0, 0.5, 0.8], 'radius': 0.30}
                    }
                }
            device: torch device ('cuda' or 'cpu')
        """
        self.device = device
        self._workspace_mask = workspace_mask
        self._center_xyz = center_xyz
        self._radius = radius
        self._workspace_definition = workspace_definition

        # Cache for generated mask
        self._cached_mask = None
        self._cache_key = None

    def _generate_mask(self, origin, resolution, size_x, size_y, size_z):
        """
        Generate or retrieve workspace mask.

        Args:
            origin, resolution, size_x, size_y, size_z: voxel grid parameters

        Returns:
            VoxelMask: Workspace mask
        """
        # Create cache key
        cache_key = (origin.x, origin.y, origin.z, resolution, size_x, size_y, size_z)

        # Return cached mask if parameters haven't changed
        if self._cached_mask is not None and self._cache_key == cache_key:
            return self._cached_mask

        # Priority 1: Use provided mask directly
        if self._workspace_mask is not None:
            mask = self._workspace_mask

        # Priority 2: Create from definition
        elif self._workspace_definition is not None:
            mask = VoxelMask.from_definition(
                self._workspace_definition,
                origin, resolution, size_x, size_y, size_z,
                self.device
            )

        # Priority 3: Create simple circular workspace
        elif self._center_xyz is not None:
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

        else:
            raise ValueError("No workspace definition provided")

        # Cache the result
        self._cached_mask = mask
        self._cache_key = cache_key

        return mask

    def compute_quality_score(self, reachability_map, origin, resolution, size_x, size_y, size_z):
        """
        Compute quality score Q: average reachability within the workspace.

        Args:
            reachability_map: torch.Tensor on GPU, shape (size_x, size_y, size_z)
            origin, resolution, size_x, size_y, size_z: voxel grid parameters

        Returns:
            float: Quality score (mean of reachability values inside workspace)
        """
        mask = self._generate_mask(origin, resolution, size_x, size_y, size_z)
        return mask.compute_mean(reachability_map)

    def apply_mask(self, reachability_map, origin, resolution, size_x, size_y, size_z):
        """
        Apply the workspace mask to a reachability map (set outside values to 0).

        Args:
            reachability_map: torch.Tensor on GPU, shape (size_x, size_y, size_z)
            origin, resolution, size_x, size_y, size_z: voxel grid parameters

        Returns:
            torch.Tensor: Masked reachability map (same shape, on GPU)
        """
        mask = self._generate_mask(origin, resolution, size_x, size_y, size_z)
        # Apply inverse mask (set voxels outside workspace to 0)
        inverted_mask = mask.invert()
        return inverted_mask.apply_to_voxelmap(reachability_map, masked_value=0.0)

    def get_masked_values(self, reachability_map, origin, resolution, size_x, size_y, size_z):
        """
        Extract reachability values inside the workspace.

        Args:
            reachability_map: torch.Tensor on GPU, shape (size_x, size_y, size_z)
            origin, resolution, size_x, size_y, size_z: voxel grid parameters

        Returns:
            torch.Tensor: 1D tensor with values inside workspace
        """
        mask = self._generate_mask(origin, resolution, size_x, size_y, size_z)
        return mask.extract_from_voxelmap(reachability_map)

    def update_center(self, new_center_xyz):
        """
        Update workspace center (for circular workspace only).

        Invalidates cache.

        Args:
            new_center_xyz: tuple (x, y, z) in meters
        """
        self._center_xyz = new_center_xyz
        self._cached_mask = None
        self._cache_key = None

    def update_workspace_mask(self, new_mask):
        """
        Update workspace with a new VoxelMask.

        Invalidates cache.

        Args:
            new_mask: VoxelMask instance
        """
        self._workspace_mask = new_mask
        self._cached_mask = None
        self._cache_key = None

    def update_workspace_definition(self, new_definition):
        """
        Update workspace with a new definition.

        Invalidates cache.

        Args:
            new_definition: dict (YAML-compatible)
        """
        self._workspace_definition = new_definition
        self._cached_mask = None
        self._cache_key = None

    def get_workspace_bounds(self, origin, resolution, size_x, size_y, size_z):
        """
        Get the bounding box of the workspace in world coordinates.

        Args:
            origin, resolution, size_x, size_y, size_z: voxel grid parameters

        Returns:
            dict: {'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max'}
                  or None if workspace is empty
        """
        mask = self._generate_mask(origin, resolution, size_x, size_y, size_z)
        bounds = mask.get_bounds()

        if bounds is None:
            return None

        # Convert indices to world coordinates
        x_min = origin.x + bounds['i_min'] * resolution
        x_max = origin.x + bounds['i_max'] * resolution
        y_min = origin.y + bounds['j_min'] * resolution
        y_max = origin.y + bounds['j_max'] * resolution
        z_min = origin.z + bounds['k_min'] * resolution
        z_max = origin.z + bounds['k_max'] * resolution

        return {
            'x_min': x_min,
            'x_max': x_max,
            'y_min': y_min,
            'y_max': y_max,
            'z_min': z_min,
            'z_max': z_max
        }

    def get_workspace_volume(self, origin, resolution, size_x, size_y, size_z):
        """
        Get the volume of the workspace in cubic meters.

        Args:
            origin, resolution, size_x, size_y, size_z: voxel grid parameters

        Returns:
            float: Volume in m³
        """
        mask = self._generate_mask(origin, resolution, size_x, size_y, size_z)
        voxel_count = mask.count()
        voxel_volume = resolution ** 3
        return voxel_count * voxel_volume
