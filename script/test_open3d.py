import open3d as o3d
import numpy as np

print('input')
bunny = o3d.data.BunnyMesh()
mesh  = o3d.io.read_triangle_mesh(bunny.path)
# fit to unit cube
mesh.scale(1 / np.max(mesh.get_max_bound() - mesh.get_min_bound()),
           center=mesh.get_center())
o3d.visualization.draw_geometries([mesh])

print('voxelization')
voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh,
                                                              voxel_size=0.5)
# o3d.visualization.draw_geometries([voxel_grid])



# create an empty voxel grid with resolution 0.02
voxel_grid_resized = o3d.geometry.VoxelGrid.create_dense(
            width=1,
            height=1,
            depth=1,
            voxel_size=0.02,
            origin=[0, 0, 0],
            color=[1.0, 0.7, 0.0])
o3d.visualization.draw_geometries([voxel_grid_resized])

#  compare voxel_grid with voxel_grid_resized to know if a voxel is occupied and update it in voxel_grid_resized
for i in range(len(voxel_grid_resized.get_voxels())):
    voxel = voxel_grid_resized.get_voxels()[i]
    coord = voxel.grid_index
    if voxel_grid.get_voxels(coord):
        voxel_grid_resized.get_voxels()[i].color = [1, 0, 0]
    else:
        voxel_grid_resized.get_voxels()[i].color = [0, 1, 0]
o3d.visualization.draw_geometries([voxel_grid_resized])


# print('input')
# N = 2000

# armadillo = o3d.data.ArmadilloMesh()
# mesh = o3d.io.read_triangle_mesh(armadillo.path)
# pcd = mesh.sample_points_poisson_disk(N)
# # fit to unit cube
# pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()),
#           center=pcd.get_center())
# pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
# o3d.visualization.draw_geometries([pcd])

# print('octree division')
# octree = o3d.geometry.Octree(max_depth=4)
# octree.convert_from_point_cloud(pcd, size_expand=0.1)
# o3d.visualization.draw_geometries([octree])

