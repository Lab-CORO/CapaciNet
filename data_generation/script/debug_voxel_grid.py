import open3d as o3d

def debug_voxel_grid():
    pcd = o3d.io.read_point_cloud("/home/ros2_ws/marker_voxel_grid.ply")
    # add coord mesh
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
    # voxel grid
    voxel_size = 0.05
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)
    o3d.visualization.draw_geometries([mesh_frame, voxel_size])

if __name__ == "__main__":
    debug_voxel_grid()