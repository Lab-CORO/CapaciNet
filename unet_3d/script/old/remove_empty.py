import numpy as np
from load_data import VoxelDataset
import open3d as o3d
import h5py
import os
import sys
import json
import shutil


# Get all files
datas = VoxelDataset(root_dir='/workspace/capacitynet/data/data')


# for all files in the samples
for i, sample_path in enumerate(datas.sample_paths):
    info_file = os.path.join(sample_path, 'info.json')
    with open(info_file, 'r') as f:
        try:
            info = json.load(f)
        except:
            shutil.rmtree(sample_path)
    reachability_file = os.path.join(sample_path, 'reachability_map.npz')

    try:
        datas.load_data(reachability_file)
    except:
        # print("none file")
        shutil.rmtree(sample_path)

    voxel_file = os.path.join(sample_path, 'voxel_grid.npz')
    try:
        datas.load_data(voxel_file)
    except:
        # print("none file")
        shutil.rmtree(sample_path)

    # check if file info is empty
