"""
Augmentation par rotation en Z (cval=0).
"""
import numpy as np
from scipy.ndimage import rotate


class RotationAugmenter:
    """Augments voxel data by applying random Z-axis rotations."""

    def __init__(self, n_copies=3, angle_spectrum=180, seed=42):
        """
        Args:
            n_copies (int): Number of rotated copies to generate per sample.
            angle_spectrum (float): Max rotation angle in degrees
                                    (uniform draw in [-angle_spectrum, +angle_spectrum]).
            seed (int): Random seed for reproducibility.
        """
        self.n_copies = n_copies
        self.angle_spectrum = angle_spectrum
        self._rng = np.random.RandomState(seed)

    def augment(self, voxel_grid, reachability_map):
        """
        Yields randomly rotated copies of the input maps.

        Args:
            voxel_grid (np.ndarray): 3D voxel grid.
            reachability_map (np.ndarray): 3D reachability map.

        Yields:
            tuple: (rotated voxel_grid, rotated reachability_map)
        """
        for _ in range(self.n_copies):
            angle = self._rng.uniform(-self.angle_spectrum, self.angle_spectrum)
            vg_rot = rotate(voxel_grid, angle, axes=(1, 0), reshape=False, order=1, mode="constant", cval=0)
            rm_rot = rotate(reachability_map, angle, axes=(1, 0), reshape=False, order=0, mode="constant", cval=0)
            yield vg_rot, rm_rot
