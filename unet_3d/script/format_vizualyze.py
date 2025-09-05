#  here, we get a file and reformat to label and input

import h5py
import numpy as np


def copy_dataset_with_attrs(src_dset, dest_group, new_name):
    """Copy a dataset and all its attributes into a new dataset inside dest_group."""
    # Copy data
    data = src_dset[()]
    new_dset = dest_group.create_dataset(new_name, data=data)

    # Copy attributes
    for attr_name, attr_value in src_dset.attrs.items():
        new_dset.attrs[attr_name] = attr_value

    return new_dset

# Input file paths
file1_path = "/workspace/capacitynet/data/test/11_08_2025_14_19_1_group_6.h5"   # contains /label and /raw
file2_path = "/workspace/capacitynet/data/test/11_08_2025_14_19_1_group_6_predictions.h5"   # contains /prediction

attributes = {
     "voxel_size": 0.02,
     "origine_y": -1.5,
 }


# Output file path
output_path = "merged.h5"

with h5py.File(file1_path, "r") as f1, \
     h5py.File(file2_path, "r") as f2, \
     h5py.File(output_path, "w") as fout:

    # Create the group structure
    group0 = fout.create_group("/group/0")

    # Copy /raw from file1 into /group/0/voxel_grid
    copy_dataset_with_attrs(f1["/raw"], group0, "voxel_grid")

    # Copy /prediction from file2 into /group/0/reachability_map
    copy_dataset_with_attrs(f2["/predictions"], group0, "reachability_map")


#  with h5py.File(file_path, "r+") as f:
    dset_path = "group/0/reachability_map"
    if dset_path in fout:
        dset = fout[dset_path]
        for key, value in attributes.items():
            dset.attrs[key] = value
        print(f"Attributes added to {dset_path}")
    else:
        print(f"Dataset '{dset_path}' not found in file.")


print(f"Fusion complete! Saved to {output_path}")
