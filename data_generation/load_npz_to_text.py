#!/usr/bin/env python3
"""
Script to load NPZ file and write its data to a text file.
"""

import numpy as np
import os
import pickle
import zipfile

def load_npz_to_text(npz_path, output_txt_path):
    """
    Load an NPZ file and write its contents to a text file.

    Args:
        npz_path: Path to the input NPZ file
        output_txt_path: Path to the output text file
    """
    # Try to load the file - it might be NPZ or a raw numpy array
    print(f"Loading file: {npz_path}")

    try:
        # First try as NPZ (compressed archive)
        data = np.load(npz_path, allow_pickle=True)
        is_npz = True
    except (pickle.UnpicklingError, ValueError, zipfile.BadZipFile):
        # If that fails, try as raw binary numpy file
        print("Not a valid NPZ file, trying as raw numpy array...")
        try:
            data = np.load(npz_path, allow_pickle=False)
            is_npz = False
        except Exception as e:
            print(f"Failed to load as numpy array: {e}")
            print("Attempting to load as raw binary data...")
            # Last resort: load as raw bytes and interpret
            with open(npz_path, 'rb') as f:
                # Skip the header bytes and try to interpret the data
                raw_data = np.fromfile(f, dtype=np.float64)
                data = {'raw_data': raw_data}
                is_npz = True

    # Open output text file
    with open(output_txt_path, 'w') as f:
        f.write(f"File: {npz_path}\n")
        f.write("=" * 80 + "\n\n")

        # Handle NPZ vs single array
        if is_npz and hasattr(data, 'files'):
            # List all arrays in the NPZ file
            f.write(f"Number of arrays: {len(data.files)}\n")
            f.write(f"Array names: {data.files}\n\n")
            f.write("=" * 80 + "\n\n")
            array_names = data.files
        elif isinstance(data, dict):
            f.write(f"Number of arrays: {len(data)}\n")
            f.write(f"Array names: {list(data.keys())}\n\n")
            f.write("=" * 80 + "\n\n")
            array_names = data.keys()
        else:
            # Single array
            f.write("Single array file\n\n")
            f.write("=" * 80 + "\n\n")
            array_names = ['data']
            data = {'data': data}

        # Write each array's data
        for array_name in array_names:
            array = data[array_name]
            f.write(f"Array: {array_name}\n")
            f.write(f"Shape: {array.shape}\n")
            f.write(f"Dtype: {array.dtype}\n")
            f.write(f"Size: {array.size} elements\n")

            # Write statistics for numeric arrays
            if np.issubdtype(array.dtype, np.number):
                f.write(f"Min: {np.min(array)}\n")
                f.write(f"Max: {np.max(array)}\n")
                f.write(f"Mean: {np.mean(array)}\n")
                f.write(f"Std: {np.std(array)}\n")

            f.write("\nData:\n")
            f.write("-" * 80 + "\n")

            # Check if this is raw pose data (format: size_t + N * 7 doubles per pose)
            # Each pose: position(x,y,z) + orientation(x,y,z,w)
            if array_name == 'raw_data' and array.dtype == np.float64:
                # First element should be the size (but stored as float64)
                # Skip it and reshape the rest as (N, 7)
                try:
                    # The data is stored as: [size, pose1_x, pose1_y, pose1_z, pose1_qx, pose1_qy, pose1_qz, pose1_qw, pose2_x, ...]
                    # We need to skip the first value and reshape
                    poses_data = array[1:].reshape(-1, 7)
                    positions = poses_data[:, :3]  # Extract x, y, z columns

                    # Filter positions outside the cube [-1, 1] in each dimension
                    outside_mask = (np.abs(positions[:, 0]) > 1.50) | \
                                   (np.abs(positions[:, 1]) > 1.50) | \
                                   (np.abs(positions[:, 2]) > 1.50)

                    positions_outside = positions[outside_mask]
                    poses_outside = poses_data[outside_mask]

                    f.write(f"Total poses: {len(poses_data)}\n")
                    f.write(f"Positions outside 2x2x2m cube: {np.sum(outside_mask)} / {len(positions)}\n")
                    f.write(f"Percentage outside: {100 * np.sum(outside_mask) / len(positions):.2f}%\n\n")

                    if len(positions_outside) > 0:
                        f.write("Positions outside cube (x, y, z):\n")
                        if len(positions_outside) > 1000:
                            f.write(f"[Too many positions - showing first 1000]\n\n")
                            f.write(str(positions_outside[:1000]) + "\n")
                        else:
                            f.write(str(positions_outside) + "\n")
                    else:
                        f.write("No positions found outside the cube.\n")
                except ValueError as e:
                    f.write(f"Could not reshape data as poses: {e}\n")
                    f.write(str(array) + "\n")
            else:
                # Write array data (limit output for very large arrays)
                if array.size > 100000:
                    f.write(f"[Array too large - showing first 100 and last 100 elements]\n\n")
                    f.write("First 100 elements:\n")
                    f.write(str(array.flat[:100]) + "\n\n")
                    f.write("Last 100 elements:\n")
                    f.write(str(array.flat[-100:]) + "\n")
                else:
                    f.write(str(array) + "\n")

            f.write("\n" + "=" * 80 + "\n\n")

    print(f"Data written to: {output_txt_path}")

    # Print summary to console
    print("\nSummary:")
    if hasattr(data, 'files'):
        print(f"  Arrays in file: {len(data.files)}")
        for array_name in data.files:
            array = data[array_name]
            print(f"  - {array_name}: shape={array.shape}, dtype={array.dtype}")
    elif isinstance(data, dict):
        print(f"  Arrays in file: {len(data)}")
        for array_name, array in data.items():
            print(f"  - {array_name}: shape={array.shape}, dtype={array.dtype}")
    else:
        print(f"  Single array: shape={data.shape}, dtype={data.dtype}")

if __name__ == "__main__":
    # Input NPZ file path
    npz_file = "/home/ros2_ws/src/CapaciNet/data_generation/data/master_ik_data0.1.npz"

    # Output text file path
    output_file = "/home/ros2_ws/src/CapaciNet/data_generation/data/master_ik_data0.1.txt"

    # Check if NPZ file exists
    if not os.path.exists(npz_file):
        print(f"Error: NPZ file not found: {npz_file}")
        exit(1)

    # Load and write data
    load_npz_to_text(npz_file, output_file)

    print("\nDone!")
