import h5py
import numpy as np
import os

def load_voxel_from_hdf5(filepath, group_path="/group/0", dataset_name=["reachability_map", "voxel_grid"]):
    """
    Load a 3D voxel dataset from an HDF5 file.

    Args:
        filepath (str): Path to the HDF5 file.
        group_path (str): Group inside the file (e.g., "/group/0").
        dataset_name (str): Dataset name inside the group.

    Returns:
        np.ndarray: Loaded voxel grid as a NumPy array.
    """
    with h5py.File(filepath, 'r') as f:
        reachability_map_path = f"{group_path}/{dataset_name[0]}"
        if reachability_map_path not in f:
            return False
            raise KeyError(f"Dataset not found: {reachability_map_path}")
            
        reachability_map = f[reachability_map_path][:]
        print(f"Loaded voxel grid of shape {reachability_map.shape} from {reachability_map_path}")

        # Voxel grid path
        voxel_grid_path = f"{group_path}/{dataset_name[1]}"
        if voxel_grid_path not in f:
            return False
            raise KeyError(f"Dataset not found: {voxel_grid_path}")
            
        voxel_grid = f[voxel_grid_path][:]
        print(f"Loaded voxel grid of shape {voxel_grid.shape} from {voxel_grid_path}")
        return reachability_map, voxel_grid



def upsample_map(voxel_map, old_size, old_res, new_res):
    """
    Upsample a voxel map using nearest-neighbor replication.
    
    Args:
        voxel_map (np.ndarray): 3D array of shape (old_size, old_size, old_size)
        old_size (int): size of the original voxel grid
        old_res (float): original resolution
        new_res (float): desired finer resolution

    Returns:
        np.ndarray: upsampled 3D voxel map
    """
    scale = int(old_res / new_res)
    new_size = old_size * scale

    # Use numpy repeat to upscale in each dimension
    upsampled = np.repeat(np.repeat(np.repeat(voxel_map, scale, axis=0), scale, axis=1), scale, axis=2)

    print(f"Upsampled from {voxel_map.shape} to {upsampled.shape}")
    return upsampled


def save_voxel_to_hdf5(voxel_grid, reachability_map, filename, group_name="", dataset_name="reachability_map"):
    """
    Save a 3D voxel grid to an HDF5 file.
    
    Args:
        voxel_grid (np.ndarray): 3D voxel map
        filename (str): Output HDF5 file
        group_name (str): HDF5 group path
        dataset_name (str): Name of dataset inside group
    """
    with h5py.File(filename, 'w') as f:
        f.create_dataset("raw", data=voxel_grid, compression="gzip")
        f.create_dataset("label", data=reachability_map, compression="gzip")
        print(f"Saved voxel grid to {filename} in {group_name}/{dataset_name}")


# Example usage
if __name__ == "__main__":
    old_size = 38
    old_res = 0.08
    new_res = 0.02

    # for loop in the folder data
    directory = "/workspace/capacitynet/data/"

    for file_name in os.listdir(directory):
        is_fully_scan = False
        index = 0
        while (not is_fully_scan):
            try:
                reachability_map, voxel_grid = load_voxel_from_hdf5(directory + file_name, group_path=f"/group/{index}")
                is_fully_scan = False
                
                # Upsample
                upsampled_voxel_map = upsample_map(voxel_grid, old_size, old_res, new_res)
                upsampled_reachability_map = upsample_map(reachability_map, old_size, old_res, new_res)

                # Save as HDF5
                file_name_split = os.path.split(file_name)
                
                output_filename = directory + "data_format/" + file_name[:-4] + f"_group_{index}.h5"

                save_voxel_to_hdf5(upsampled_voxel_map, upsampled_reachability_map, output_filename)
                index +=1
            except:
                is_fully_scan = True
                index = 0
