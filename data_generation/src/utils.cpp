//
// Created by will on 06/06/24.
//

#include "../include/utils.h"



// write a class to add math operation as round_to_decimals




double utils::round_to_decimals(double value, int decimals) {
    double factor = std::pow(10, decimals);
    double res = round(value * factor) / factor;
    if (res >1000) {
        std::cout << "res: " << res << std::endl;
        std::cout << "factor: " << factor << std::endl;
        std::cout << "value: " << value << std::endl;
    }
    return res;
}


bool utils::split_data(std::vector<geometry_msgs::msg::Pose>& data, size_t max_batch_size, std::vector<std::vector<geometry_msgs::msg::Pose>>& batches) {
    if (max_batch_size == 0) {
        std::cerr << "Batch size must be greater than 0." << std::endl;
        return false;
    }
    // Get the number of batch 
    size_t num_batche = std::size(data) / max_batch_size;

    
    // Compute reste and add an absurde value
    size_t reste =   std::size(data) - (num_batche * max_batch_size) ;
    // Absurde value
    geometry_msgs::msg::Pose reste_pose;
    reste_pose.position.x = 1225.0;
    reste_pose.position.y = 1225.0;
    reste_pose.position.z = 1225.0;

    if (reste != 0){
        for (size_t r = 0; r < max_batch_size - reste ; r++){
             data.push_back(reste_pose);
        } 
     }
    num_batche = std::size(data) / max_batch_size;
    batches.clear();
    for (size_t i = 0; i < num_batche; ++i) {
        size_t start_idx = i * max_batch_size;
        size_t end_idx = i * max_batch_size + max_batch_size;
        std::vector<geometry_msgs::msg::Pose> batch(data.begin() + start_idx, data.begin() + end_idx);
        batches.push_back(batch);
    }

    return true;
}


bool utils::save_poses_to_file(const std::string& filename, const std::vector<geometry_msgs::msg::Pose>& poses) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    size_t size = poses.size();
    file.write(reinterpret_cast<const char*>(&size), sizeof(size));

    for (const auto& pose : poses) {
        file.write(reinterpret_cast<const char*>(&pose.position.x), sizeof(pose.position.x));
        file.write(reinterpret_cast<const char*>(&pose.position.y), sizeof(pose.position.y));
        file.write(reinterpret_cast<const char*>(&pose.position.z), sizeof(pose.position.z));
        file.write(reinterpret_cast<const char*>(&pose.orientation.x), sizeof(pose.orientation.x));
        file.write(reinterpret_cast<const char*>(&pose.orientation.y), sizeof(pose.orientation.y));
        file.write(reinterpret_cast<const char*>(&pose.orientation.z), sizeof(pose.orientation.z));
        file.write(reinterpret_cast<const char*>(&pose.orientation.w), sizeof(pose.orientation.w));
    }
    return true;
}

bool utils::saveVecToNpz(const std::string& filename, const std::vector<std::array<double, 4>>& data) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    size_t size = data.size();
    file.write(reinterpret_cast<const char*>(&size), sizeof(size));

    for (const auto& entry : data) {
        file.write(reinterpret_cast<const char*>(entry.data()), sizeof(double) * 4);
    }

    return true;
}

bool utils::load_poses_from_file(const std::string& filename, std::vector<geometry_msgs::msg::Pose>& poses) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return false;
    }

    size_t size;
    file.read(reinterpret_cast<char*>(&size), sizeof(size));

    poses.resize(size);
    for (auto& pose : poses) {
        file.read(reinterpret_cast<char*>(&pose.position.x), sizeof(pose.position.x));
        file.read(reinterpret_cast<char*>(&pose.position.y), sizeof(pose.position.y));
        file.read(reinterpret_cast<char*>(&pose.position.z), sizeof(pose.position.z));
        file.read(reinterpret_cast<char*>(&pose.orientation.x), sizeof(pose.orientation.x));
        file.read(reinterpret_cast<char*>(&pose.orientation.y), sizeof(pose.orientation.y));
        file.read(reinterpret_cast<char*>(&pose.orientation.z), sizeof(pose.orientation.z));
        file.read(reinterpret_cast<char*>(&pose.orientation.w), sizeof(pose.orientation.w));
    }

    return true;
}

bool utils::saveToHDF5(const std::map<utils::QuantizedPoint3D, double> &data,
            const std::vector<std::array<double, 4>> &voxel_grid,
            float voxel_size,
            int (&voxel_grid_sizes)[3],
            double (&voxel_grid_origin)[3],
            std::shared_ptr<HighFive::File> data_file,
            int dataset_id
            )
        {
            using namespace HighFive;

            
            vector<vector<vector<double>>> voxel_grid_data(voxel_grid_sizes[0], 
                                            vector<vector<double>>(voxel_grid_sizes[1], 
                                            vector<double>(voxel_grid_sizes[2], 
                                            1.0))); // initial value is 1.0 (means free voxel)
            
            double resolution = voxel_size;
            double origine = voxel_grid_origin[0];


            double rm_size = voxel_grid_sizes[0]; 
            
            for (size_t idx = 0; idx < voxel_grid.size(); ++idx) {
                size_t x = idx / (voxel_grid_sizes[1] * voxel_grid_sizes[2]);
                size_t y = (idx / voxel_grid_sizes[2]) % voxel_grid_sizes[1];
                size_t z = idx % voxel_grid_sizes[2];
                voxel_grid_data[x][y][z] = 1 - voxel_grid[idx][3];
            }
            
            // create a rm map with only 0
            vector<vector<vector<double>>> rm_data(rm_size, 
                                            vector<vector<double>>(rm_size, 
                                            vector<double>(rm_size, 
                                            0.0))); // initial value is 0 (means no reach)
            for(const auto &pose : data) {
                int idx = static_cast<int> (pose.first.x + static_cast<int>(rm_size / 2));
                int idy = static_cast<int> (pose.first.y + static_cast<int>(rm_size / 2));
                int idz = static_cast<int> (pose.first.z + static_cast<int>(rm_size / 2));
                rm_data[idx][idy][idz] = pose.second;
            }
            std::string dataset_id_s = std::to_string(dataset_id);

            // Create the group structure if it does not exist
            auto group = data_file->createGroup("/group/" + dataset_id_s);
            // create voxel map dataset
            std::vector<size_t> dims_voxel_grid{(size_t)voxel_grid_sizes[0], (size_t)voxel_grid_sizes[1], (size_t)voxel_grid_sizes[2]};
            DataSet dataset_voxelgrid = group.createDataSet<double>("voxel_grid", DataSpace(dims_voxel_grid));
            dataset_voxelgrid.write(voxel_grid_data);
            // add voxel map attribut
            dataset_voxelgrid.createAttribute<double>("origine_x", voxel_grid_origin[0]);
            dataset_voxelgrid.createAttribute<double>("origine_y", voxel_grid_origin[1]);
            dataset_voxelgrid.createAttribute<double>("origine_z", voxel_grid_origin[2]);
            dataset_voxelgrid.createAttribute<double>("voxel_size", voxel_size);
            dataset_voxelgrid.createAttribute<double>("voxel_grid_size_x", voxel_grid_sizes[0]);
            dataset_voxelgrid.createAttribute<double>("voxel_grid_size_y", voxel_grid_sizes[1]);
            dataset_voxelgrid.createAttribute<double>("voxel_grid_size_z", voxel_grid_sizes[2]);


            // Now create datasets within these groups
            std::vector<size_t> dims_rm_grid{(size_t)rm_size, (size_t)rm_size, (size_t)rm_size};
            DataSet dataset_data = group.createDataSet<double>("reachability_map", DataSpace(dims_rm_grid));
            dataset_data.write(rm_data);
            // add voxel map attribut
            dataset_data.createAttribute<double>("origine_x", origine);
            dataset_data.createAttribute<double>("origine_y", origine);
            dataset_data.createAttribute<double>("origine_z", origine);
            dataset_data.createAttribute<double>("voxel_size", resolution);
            dataset_data.createAttribute<double>("voxel_grid_size_x", rm_size);
            dataset_data.createAttribute<double>("voxel_grid_size_y", rm_size);
            dataset_data.createAttribute<double>("voxel_grid_size_z", rm_size);

            return true;
}

bool utils::loadFromHDF5(const std::string& filename,
                std::map<utils::QuantizedPoint3D, double> &reachability_map,
                double &resolution,
                std::array<double, 3> &voxel_grid_origin,
                std::array<int, 3> &voxel_grid_sizes,
                int group_id)
{
    using namespace HighFive;

    try {
        // Open the HDF5 file in read-only mode
        File file(filename, File::ReadOnly);

        // Construct group path
        std::string group_path = "/group/" + std::to_string(group_id);

        // Check if group exists
        if (!file.exist(group_path)) {
            std::cerr << "Group " << group_path << " does not exist in file " << filename << std::endl;
            return false;
        }

        // Open the group
        Group group = file.getGroup(group_path);

        // Check if reachability_map dataset exists
        if (!group.exist("reachability_map")) {
            std::cerr << "Dataset 'reachability_map' does not exist in group " << group_path << std::endl;
            return false;
        }

        // Open the reachability_map dataset
        DataSet dataset = group.getDataSet("reachability_map");

        // Read attributes
        double origine_x, origine_y, origine_z;
        double voxel_size;
        double size_x, size_y, size_z;

        dataset.getAttribute("origine_x").read(origine_x);
        dataset.getAttribute("origine_y").read(origine_y);
        dataset.getAttribute("origine_z").read(origine_z);
        dataset.getAttribute("voxel_size").read(voxel_size);
        dataset.getAttribute("voxel_grid_size_x").read(size_x);
        dataset.getAttribute("voxel_grid_size_y").read(size_y);
        dataset.getAttribute("voxel_grid_size_z").read(size_z);

        // Set output parameters
        resolution = voxel_size;
        voxel_grid_origin[0] = origine_x;
        voxel_grid_origin[1] = origine_y;
        voxel_grid_origin[2] = origine_z;
        voxel_grid_sizes[0] = static_cast<int>(size_x);
        voxel_grid_sizes[1] = static_cast<int>(size_y);
        voxel_grid_sizes[2] = static_cast<int>(size_z);

        // Read the 3D reachability map data
        std::vector<std::vector<std::vector<double>>> rm_data;
        dataset.read(rm_data);

        // Convert 3D array to map with QuantizedPoint3D keys
        reachability_map.clear();
        int rm_size = voxel_grid_sizes[0];

        for (int idx = 0; idx < rm_size; ++idx) {
            for (int idy = 0; idy < rm_size; ++idy) {
                for (int idz = 0; idz < rm_size; ++idz) {
                    double value = rm_data[idx][idy][idz];

                    // Only store non-zero values to save memory
                    if (value > 1e-10) {
                        // Convert back from array indices to quantized coordinates
                        int quantized_x = idx - (rm_size / 2);
                        int quantized_y = idy - (rm_size / 2);
                        int quantized_z = idz - (rm_size / 2);

                        // Create QuantizedPoint3D with the pre-quantized coordinates
                        // We need to reverse the quantization to get the original position
                        double world_x = quantized_x * resolution;
                        double world_y = quantized_y * resolution;
                        double world_z = quantized_z * resolution;

                        QuantizedPoint3D point(world_x, world_y, world_z, resolution);
                        reachability_map[point] = value;
                    }
                }
            }
        }

        std::cout << "Successfully loaded " << reachability_map.size()
                  << " reachable voxels from " << filename
                  << " (group " << group_id << ")" << std::endl;
        std::cout << "Resolution: " << resolution << "m" << std::endl;
        std::cout << "Origin: [" << origine_x << ", " << origine_y << ", " << origine_z << "]" << std::endl;
        std::cout << "Grid size: [" << voxel_grid_sizes[0] << ", "
                  << voxel_grid_sizes[1] << ", " << voxel_grid_sizes[2] << "]" << std::endl;

        return true;

    } catch (const HighFive::Exception& e) {
        std::cerr << "HDF5 error while loading " << filename << ": " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error while loading " << filename << ": " << e.what() << std::endl;
        return false;
    }
}
