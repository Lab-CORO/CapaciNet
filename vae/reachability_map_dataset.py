import os
import json
import numpy as np
import torch
from torch.utils.data import Dataset
import h5py

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


class ReachabilityMapDataset(Dataset):
    def __init__(self, root_dir, transform=None):
        """
        Args:
            root_dir (str): Path to the dataset root directory.
            transform (callable, optional): Optional transform to be applied
                on a sample (e.g., converting numpy arrays to torch tensors or augmentations).
        """
        self.root_dir = root_dir
        self.transform = transform
        self.sample_paths = []

        # get path from hdf5 root_dir
        with h5py.File(self.root_dir, "r") as f:
            self.sample_paths = list(f.keys())
            # print(self.sample_paths)
        

    def __len__(self):
        return len(self.sample_paths)   


    def load_data(self, filename):

        # with h5py.File(self.root_dir, "r") as f:
        #     # Read the first 8 bytes for the record count (assuming 64-bit size_t)
        return None
    

    
    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        sample_path = self.sample_paths[idx]

        with h5py.File(self.root_dir, "r") as f:

        # Load the reachability_map.npz file
            reachability_map =  f[sample_path+'/reachability_map']#self.load_data(reachability_file)
            voxel_map =  f[sample_path+'/voxel_map']
            # get attribut
            resolution = reachability_map.attrs["resolution"]
            voxel_grid_origin = reachability_map.attrs["voxel_grid_origin"] 
            voxel_grid_sizes = reachability_map.attrs["voxel_grid_sizes"]
            reachability_map = reachability_map[:]
            voxel_map = voxel_map[:]
            # voxel_map = np.asarray([voxel_map[:]])

        # If no external transform is provided, convert arrays to torch tensors
        if self.transform:
            voxel_grid = self.transform(voxel_grid)
        else:
            voxel_grid = torch.from_numpy(voxel_map.astype(np.float32))
            reachability_map = torch.from_numpy(reachability_map)

        sample = {
            'resolution': resolution,
            'voxel_grid_origin': voxel_grid_origin,
            'voxel_grid_sizes': voxel_grid_sizes,
            'reachability_map': reachability_map.to(DEVICE),
            'voxel_grid': voxel_grid.to(DEVICE)
        }

        return sample


# dataset = ReachabilityMapDataset(root_dir='/workspace/capacitynet/data/data.h5')
# from torch.utils.data import DataLoader
# dataloader = DataLoader(dataset, batch_size=10, shuffle=True)
# for i, sample in enumerate(dataloader):
#     print(sample['reachability_map'].size())
    # input("valid")