"""
This file converts generated data into data files usable by the unet3d project.
"""
import argparse
import os
import pathlib
import random
import shutil

import h5py
import numpy as np
from tqdm import tqdm

from augment_rotation_z import RotationAugmenter


class DataFormatter:
    """Converts raw HDF5 voxel files into unet3d-compatible format.

    Handles optional upsampling to a target resolution and optional
    data augmentation via rotation.
    """

    def __init__(self, input_dir, output_dir, new_res=0.02, augmenter=None, crop=None):
        """
        Args:
            input_dir (str): Directory containing input HDF5 files.
            output_dir (str): Directory where output HDF5 files are saved.
            new_res (float): Target voxel resolution. Files with a different
                             resolution are upsampled to match.
            augmenter (RotationAugmenter | None): Optional augmenter. If provided,
                                                   augmented copies are saved alongside each original.
            crop (tuple | None): ((x0, x1), (y0, y1), (z0, z1)) indices to crop each volume,
                                  or None to skip cropping.
        """
        self.input_dir = input_dir
        self.output_dir = output_dir
        self.new_res = new_res
        self.augmenter = augmenter
        self.crop = crop

    def _list_groups(self, filepath):
        """Return the sorted list of group indices available in the file.

        Returns:
            list[str]: Sorted group keys (e.g. ['0', '1', '2']).
        """
        with h5py.File(filepath, 'r') as f:
            if "group" not in f:
                return []
            return sorted(f["group"].keys(), key=int)

    def _load_group(self, filepath, group_path):
        """Load voxel_grid, reachability_map and attributes from an HDF5 group.

        Returns:
            tuple: (reachability_map, voxel_grid, attrs) or False if voxel_grid is missing.

        Raises:
            KeyError: If the reachability_map dataset is not found.
        """
        with h5py.File(filepath, 'r') as f:
            reachability_map_path = f"{group_path}/reachability_map"
            if reachability_map_path not in f:
                raise KeyError(f"Dataset not found: {reachability_map_path}")
            reachability_map = f[reachability_map_path][:]

            voxel_grid_path = f"{group_path}/voxel_grid"
            if voxel_grid_path not in f:
                return False
            voxel_grid = f[voxel_grid_path][:]

            attrs = dict(f[reachability_map_path].attrs)

        return reachability_map, voxel_grid, attrs

    def _upsample_map(self, voxel_map, voxel_size):
        """Upsample a 3D map using nearest-neighbor replication.

        Args:
            voxel_map (np.ndarray): 3D array to upsample.
            voxel_size (float): Current resolution of the map.

        Returns:
            np.ndarray: Upsampled 3D array.
        """
        scale = int(round(voxel_size / self.new_res))
        upsampled = np.repeat(np.repeat(np.repeat(voxel_map, scale, axis=0), scale, axis=1), scale, axis=2)
        print(f"Upsampled from {voxel_map.shape} to {upsampled.shape}")
        return upsampled

    def _upsample_if_needed(self, voxel_grid, reachability_map, attrs):
        """Upsample both maps if their voxel_size differs from self.new_res.

        Returns:
            tuple: (voxel_grid, reachability_map, attrs) with updated values.
        """
        attrs = dict(attrs)
        voxel_size = float(attrs.get("voxel_size", self.new_res))
        if round(voxel_size, 2) != self.new_res:
            voxel_grid = self._upsample_map(voxel_grid, voxel_size)
            reachability_map = self._upsample_map(reachability_map, voxel_size)
            attrs["voxel_size"] = self.new_res
            new_size = float(voxel_grid.shape[0])
            attrs["voxel_grid_size_x"] = new_size
            attrs["voxel_grid_size_y"] = new_size
            attrs["voxel_grid_size_z"] = new_size
        return voxel_grid, reachability_map, attrs

    def _crop(self, voxel_grid, reachability_map):
        (x0, x1), (y0, y1), (z0, z1) = self.crop
        return voxel_grid[x0:x1, y0:y1, z0:z1], reachability_map[x0:x1, y0:y1, z0:z1]

    def _save(self, voxel_grid, reachability_map, filename, attrs):
        """Save voxel_grid and reachability_map to an HDF5 file in unet3d format.

        Args:
            voxel_grid (np.ndarray): 3D voxel grid saved as 'raw'.
            reachability_map (np.ndarray): 3D map saved as 'label'.
            filename (str): Output file path.
            attrs (dict): Attributes to attach to both datasets.
        """
        with h5py.File(filename, 'w') as f:
            raw_dataset = f.create_dataset("raw", data=voxel_grid, compression="gzip")
            label_dataset = f.create_dataset("label", data=reachability_map, compression="gzip")
            for key, val in attrs.items():
                raw_dataset.attrs[key] = val
                label_dataset.attrs[key] = val

    def _is_already_formatted(self, filepath):
        """Return True if the file already has raw/label datasets at the top level."""
        with h5py.File(filepath, 'r') as f:
            return "raw" in f and "label" in f

    def _load_formatted(self, filepath):
        """Load raw/label from an already-formatted HDF5 file."""
        with h5py.File(filepath, 'r') as f:
            voxel_grid = f["raw"][:]
            reachability_map = f["label"][:]
            attrs = dict(f["raw"].attrs)
        return voxel_grid, reachability_map, attrs

    def run(self):
        """Process all HDF5 files in input_dir and write converted files to output_dir."""
        os.makedirs(self.output_dir, exist_ok=True)
        h5_files = [f for f in os.listdir(self.input_dir) if f.endswith(".h5")]

        skipped = []
        already_formatted_count = 0
        for file_name in tqdm(h5_files, desc="Files", unit="file"):
            filepath = os.path.join(self.input_dir, file_name)
            stem = os.path.splitext(file_name)[0]

            try:
                if self._is_already_formatted(filepath):
                    already_formatted_count += 1
                    voxel_grid, reachability_map, attrs = self._load_formatted(filepath)
                    voxel_grid, reachability_map, attrs = self._upsample_if_needed(
                        voxel_grid, reachability_map, attrs
                    )
                    if self.crop is not None:
                        voxel_grid, reachability_map = self._crop(voxel_grid, reachability_map)
                    output_filename = os.path.join(self.output_dir, file_name)
                    self._save(voxel_grid, reachability_map, output_filename, attrs)
                    if self.augmenter is not None:
                        for aug_i, (vg_aug, rm_aug) in enumerate(self.augmenter.augment(voxel_grid, reachability_map)):
                            aug_filename = os.path.join(self.output_dir, f"{stem}_aug_{aug_i}.h5")
                            self._save(vg_aug, rm_aug, aug_filename, attrs)
                    continue

                group_keys = self._list_groups(filepath)
            except OSError as e:
                tqdm.write(f"WARNING: skipping corrupted file {file_name}: {e}")
                skipped.append(file_name)
                continue

            for index in tqdm(group_keys, desc=file_name, unit="group", leave=False):
                result = self._load_group(filepath, group_path=f"/group/{index}")
                if result is False:
                    continue
                reachability_map, voxel_grid, attrs = result

                voxel_grid, reachability_map, attrs = self._upsample_if_needed(
                    voxel_grid, reachability_map, attrs
                )

                if self.crop is not None:
                    voxel_grid, reachability_map = self._crop(voxel_grid, reachability_map)

                # Save original
                output_filename = os.path.join(self.output_dir, f"{stem}_group_{index}.h5")
                self._save(voxel_grid, reachability_map, output_filename, attrs)

                # Save augmented copies
                if self.augmenter is not None:
                    for aug_i, (vg_aug, rm_aug) in enumerate(self.augmenter.augment(voxel_grid, reachability_map)):
                        aug_filename = os.path.join(self.output_dir, f"{stem}_group_{index}_aug_{aug_i}.h5")
                        self._save(vg_aug, rm_aug, aug_filename, attrs)

        if already_formatted_count:
            print(f"Passed through {already_formatted_count} already-formatted file(s) directly.")
        if skipped:
            print(f"\nSkipped {len(skipped)} corrupted file(s): {skipped}")

    def split(self, val_ratio=0.1, seed=0):
        """Move output files into train/ and val/ subdirectories.

        Args:
            val_ratio (float): Fraction of files assigned to validation.
            seed (int): Random seed for reproducibility.
        """
        root = pathlib.Path(self.output_dir)
        files = sorted(root.glob("*.h5"))
        (root / "train").mkdir(parents=True, exist_ok=True)
        (root / "val").mkdir(parents=True, exist_ok=True)
        random.seed(seed)
        val_files = set(random.sample(files, max(1, int(len(files) * val_ratio))))
        for f in files:
            dest = root / "val" / f.name if f in val_files else root / "train" / f.name
            shutil.move(f, dest)
        print(f"Split done: {len(files) - len(val_files)} train, {len(val_files)} val")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert HDF5 voxel files to unet3d format.")
    parser.add_argument("--input_dir", type=str, default="/lustre06/project/6089348/willore/data/")
    parser.add_argument("--output_dir", type=str, default="/lustre06/project/6089348/willore/CapaciNet/unet_3d/data/")
    parser.add_argument("--split", action="store_true", help="Split output files into train/val subfolders")
    parser.add_argument("--val_ratio", type=float, default=0.1, help="Fraction of files used for validation (default: 0.1)")
    parser.add_argument("--seed", type=int, default=0, help="Random seed for train/val split (default: 0)")
    parser.add_argument("--n_copies", type=int, default=0, help="Number of augmented (rotated) copies per group (0 = no augmentation)")
    parser.add_argument("--angle_spectrum", type=float, default=180, help="Max rotation angle for augmentation in degrees (default: 180)")
    parser.add_argument("--aug_seed", type=int, default=42, help="Random seed for augmentation (default: 42)")
    parser.add_argument(
        "--crop", type=int, nargs=6, metavar=("X0", "X1", "Y0", "Y1", "Z0", "Z1"),
        default=None,
        help="Crop indices applied after upsampling: x0 x1 y0 y1 z0 z1  (e.g. --crop 12 140 12 140 20 148)"
    )
    args = parser.parse_args()

    augmenter = None
    if args.n_copies > 0:
        augmenter = RotationAugmenter(n_copies=args.n_copies, angle_spectrum=args.angle_spectrum, seed=args.aug_seed)

    crop = (
        (args.crop[0], args.crop[1]),
        (args.crop[2], args.crop[3]),
        (args.crop[4], args.crop[5]),
    ) if args.crop is not None else None

    formatter = DataFormatter(
        input_dir=args.input_dir,
        output_dir=args.output_dir,
        augmenter=augmenter,
        crop=crop,
    )
    formatter.run()

    if args.split:
        formatter.split(val_ratio=args.val_ratio, seed=args.seed)
