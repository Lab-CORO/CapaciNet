#ifndef HDF5_MANAGER_HPP
#define HDF5_MANAGER_HPP
// #include <rclcpp/rclcpp.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
#include <highfive/H5File.hpp>

#include <boost/multi_array.hpp>
// #include <highfive/highfive.hpp>

using namespace HighFive;

class DatasetH5Manager
{
public:
    // Var
    std::shared_ptr<HighFive::File> data_file_;
    // int dataset_id = 0;

    // Methods
    DatasetH5Manager(const std::string &file_path);
    // write file
    void saveToHDF5(const std::vector<std::array<double, 4>> &data, const std::vector<std::array<double, 4>> &voxel_grid, float voxel_size, int (&voxel_grid_sizes)[3], double (&voxel_grid_origin)[3]);
    // read h5 file from path with highfive
    void read_h5_file(std::string file_path);

    // get voxel map
    std::vector<std::array<double, 4>> get_voxel_map(int dataset_id);

    // get voxel parameters
    std::array<double, 3> get_voxel_parameters(int dataset_id);

    // voxel map origine
    std::array<double, 3> get_voxelmap_origine(int dataset_id);

    // get the number of dataset id in a group
    int get_dataset_size();

    double get_resolution(int dataset_id);
    // get reachability map
    std::vector<std::array<double, 4>> get_reachability_map(int dataset_id);
};

#endif //HDF5_MANAGER_HPP