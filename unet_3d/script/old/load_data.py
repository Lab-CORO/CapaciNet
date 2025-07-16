import os
import json
import numpy as np
import torch
from torch.utils.data import Dataset


class VoxelDataset(Dataset):
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
        info_json = []

        # Iterate over each session directory (e.g. "10_03_2025_21_57_30", etc.)
        for session in os.listdir(root_dir):
            session_path = os.path.join(root_dir, session)
            if os.path.isdir(session_path):
                # Iterate over each sample directory within the session folder
                for sample in os.listdir(session_path):
                    sample_path = os.path.join(session_path, sample)
                    if os.path.isdir(sample_path):
                        self.sample_paths.append(sample_path)

        # Sort for reproducibility (optional)
        self.sample_paths.sort()
    def load_data(self, filename):

        with open(filename, "rb") as f:
            # Read the first 8 bytes for the record count (assuming 64-bit size_t)
            size_array = np.frombuffer(f.read(8), dtype=np.uint64)
            if size_array.size < 1:
                print(filename)
                raise ValueError("Failed to read the number of records.")

            num_records = int(size_array[0])

            # Read the remaining bytes as double precision floats.
            # Each record has 4 doubles.
            data = np.frombuffer(f.read(), dtype=np.float64)

            expected_doubles = num_records * 4
            if data.size != expected_doubles:
                print(filename)

                raise ValueError(f"Data size mismatch: expected {expected_doubles} doubles, got {data.size}.")

            # Reshape the flat array into a 2D array with 4 columns.
            return data.reshape((num_records, 4))
        return None


    def __len__(self):
        return len(self.sample_paths)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        sample_path = self.sample_paths[idx]

        # Load the info.json file
        info_file = os.path.join(sample_path, 'info.json')
        with open(info_file, 'r') as f:
            # print(info_file)
            info = json.load(f)
            self.info_json = info
            
        # Load the reachability_map.npz file
        reachability_file = os.path.join(sample_path, 'reachability_map.npz')
        reachability_map = self.load_data(reachability_file)

        # Load the voxel_grid.npz file
        voxel_file = os.path.join(sample_path, 'voxel_grid.npz')
        voxel_grid = self.load_data(voxel_file)
        
        # If no external transform is provided, convert arrays to torch tensors
        if self.transform:
            voxel_grid = self.transform(voxel_grid)
        else:
            voxel_grid = torch.from_numpy(voxel_grid).float()
            reachability_map = torch.from_numpy(reachability_map).float()

        sample = {
            'info': info,
            'reachability_map': reachability_map,
            'voxel_grid': voxel_grid
        }

        return sample

# Example usage:
# dataset = VoxelDataset(root_dir='/workspace/capacitynet/data')
# from torch.utils.data import DataLoader
# dataloader = DataLoader(dataset, batch_size=4, shuffle=True)
# for i, sample in enumerate(dataloader):
#     print(sample['reachability_map'].size())