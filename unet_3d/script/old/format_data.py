import numpy as np
from load_data import VoxelDataset
import open3d as o3d
import h5py

import sys
np.set_printoptions(threshold=sys.maxsize)


def reformat_data(reachability_map, voxel_map):
    reachability_map = np.resize(reachability_map,(4*5428,4))
    voxel_map = np.resize(voxel_map,(voxel_map.shape[0]*voxel_map.shape[1],4))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(reachability_map[:, :3])
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,
                                                            voxel_size=0.08)

    # Get if reach point are on voxel grid:
    queries = voxel_map[:, :3]
    output = voxel_grid.check_if_included(o3d.utility.Vector3dVector(queries))
    # print(output)

    input()

def generate_commun_map():

    start, end, resolution = -1.5, 1.5, 0.08 #TODO change this as arg to fit to others

    # Create arrays for each dimension. Adding resolution to end ensures the endpoint is included.
    x = np.arange(start, end, resolution)
    y = np.arange(start, end, resolution)
    z = np.arange(start, end, resolution)

    # Create a 3D meshgrid of coordinates
    X, Y, Z = np.meshgrid(x, y, z, indexing='ij')

    # Stack the coordinate arrays into a single (N, 3) array where N is the total number of voxels.
    voxels = np.vstack((X.ravel(), Y.ravel(), Z.ravel())).T

    # Create a column of zeros for the extra data
    data = np.zeros((voxels.shape[0], 1))

    # Concatenate the voxel coordinates with the data column to form a (N, 4) array.
    commun_map = np.hstack((voxels, data))
    return commun_map

def fill_reachability_map(reachability_map,new_reachability_map):
    new_reachability_map = np.round(new_reachability_map, 2)
    voxel_index_dict = {
        (str(new_reachability_map[i, 0])+str(new_reachability_map[i, 1])+str(new_reachability_map[i, 2])): i 
        for i in range(new_reachability_map.shape[0])
    }
    # For each entry in update_data, update the corresponding voxel if it exists.
    reachability_map = np.round(reachability_map, 2)
    for row in reachability_map:
        coord = str(row[0])+str(row[1])+str(row[2])
        if coord in voxel_index_dict:
            index = voxel_index_dict[coord]
            new_reachability_map[index, 3] = row[3]

    return new_reachability_map

def fill_voxel_map(voxel_map,new_voxel_map):
    voxel_map = np.resize(voxel_map,(voxel_map.shape[0]*voxel_map.shape[1],4))
    voxel_pcd = []
    # get point that are voxel
    for point in voxel_map:
        if point[3] == 0:
            voxel_pcd.append(point[:3])
    voxel_pcd = np.array(voxel_pcd)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(voxel_pcd)
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,
                                                            voxel_size=0.08)
    # o3d.visualization.draw_geometries([voxel_grid])
    queries = new_voxel_map[:, :3]
    output = voxel_grid.check_if_included(o3d.utility.Vector3dVector(queries))

    for i, val in enumerate(output):
        new_voxel_map[i, 3] = 0 if val else 1
       

    return new_voxel_map



dataset = VoxelDataset(root_dir='/workspace/capacitynet/data/data/data')
from torch.utils.data import DataLoader
dataloader = DataLoader(dataset, batch_size=4, shuffle=True)

with h5py.File("data.h5", "w") as f:
        for i, sample in enumerate(dataloader):
            # print reahcabilitymap and voxle grid in a file
            commun_map = generate_commun_map()

            vox = fill_voxel_map(sample['voxel_grid'], commun_map.copy())
            vox = np.resize(vox,(38, 38, 38, 4))

            reachability_map =sample['reachability_map']
            reachability_map = np.resize(reachability_map,(4*5428,4))
            print(len(reachability_map))
            
            res = fill_reachability_map(reachability_map, commun_map.copy())
            

            res = np.resize(res,(38, 38, 38, 4))

            # keep only the third value (we can get all others by info)
            vox = vox[:,:,:,3]
            res = res[:,:,:,3]

            metadata = {
            "resolution": sample['info']["resolution"],
            "voxel_grid_origin": [-1.5, -1.5, -1.5],
            "voxel_grid_sizes": [38, 38, 38]
            }


            # save the file to a HDF5 file
            dataset_name = f"array_{i}"
            dset1 = f.create_dataset(dataset_name+"/voxel_map", data=vox)
            dset2 = f.create_dataset(dataset_name+"/reachability_map", data=res)
            # dset1 = f.create_dataset(dataset_name+"/info", shape=len(dict))

            # dset1.attrs["voxel_map"] = vox
            # dset1.attrs["reachability_map"] = res
            dset1.attrs["resolution"] = metadata["resolution"]
            dset1.attrs["voxel_grid_origin"] = metadata["voxel_grid_origin"]
            dset1.attrs["voxel_grid_sizes"] = metadata["voxel_grid_sizes"]

            dset2.attrs["resolution"] = metadata["resolution"]
            dset2.attrs["voxel_grid_origin"] = metadata["voxel_grid_origin"]
            dset2.attrs["voxel_grid_sizes"] = metadata["voxel_grid_sizes"]

