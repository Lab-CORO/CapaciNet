import numpy as np
import sys
np.set_printoptions(threshold=sys.maxsize)

start, end, resolution = -2, 2, 0.08

# Create arrays for each dimension. Adding resolution to end ensures the endpoint is included.
x = np.arange(start, end + resolution, resolution)
y = np.arange(start, end + resolution, resolution)
z = np.arange(start, end + resolution, resolution)

# Create a 3D meshgrid of coordinates
X, Y, Z = np.meshgrid(x, y, z, indexing='ij')

# Stack the coordinate arrays into a single (N, 3) array where N is the total number of voxels.
voxels = np.vstack((X.ravel(), Y.ravel(), Z.ravel())).T

# Create a column of zeros for the extra data
data = np.zeros((voxels.shape[0], 1))

# Concatenate the voxel coordinates with the data column to form a (N, 4) array.
voxels_with_data = np.hstack((voxels, data))

print("Shape of voxel array with data:", voxels_with_data.shape)
print("Voxel coordinates with data:\n", voxels_with_data)



# --- Step 3: Update the voxel map ---
# Build a dictionary to quickly map voxel coordinates to their index in the voxel grid
voxel_index_dict = {
    (voxels_with_data[i, 0], voxels_with_data[i, 1], voxels_with_data[i, 2]): i 
    for i in range(voxels_with_data.shape[0])
}

# For each entry in update_data, update the corresponding voxel if it exists.
reachability_map = np.round(reachability_map, 3)
for row in reachability_map:
    coord = (row[0], row[1], row[2])
    if coord in voxel_index_dict:
        index = voxel_index_dict[coord]
        voxels_with_data[index, 3] = row[3]



# 1. creer un tableau 3D de la meme dimension que la reachability map 2, 2, 2 resolution 0.8
# 2. pour chaque coordonee, si il existe, prendre la valeur, sinon mettre 0
# 3. creer un autre tableaui similaire pour les voxel et comparer entre les point et la voxel map. pour occupe 0 et libre 1.

