#include "../include/hdf5_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <bits/stdc++.h>
#include <cstdlib>
#include <iostream>

using namespace std;

using namespace HighFive;

class VoxelHeatmap : public rclcpp::Node
{
public:
  // var
  std::shared_ptr<HighFive::File> data_file_;
  std::string files_path;
  // h5 manager
  // DatasetH5Manager h5_manager;

  // methods
  VoxelHeatmap(const std::string &file_path) : Node("voxel_heatmap"), files_path(file_path)
  {
    this->files_path = file_path;
    // heatmap();
    // this->h5_manager = DatasetH5Manager(file_path);
  }

  void heatmap()
  {
    // open file with hdf5
    DatasetH5Manager h5_file(this->files_path);

    // get the voxel map
    std::vector<std::array<double, 4>> voxel_map = h5_file.get_voxel_map(0);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoxelHeatmap>("/home/ros2_ws/src/capacitynet/data/22_01_2025_12_06_46.h5");
  // rclcpp::spin(node);
  node->heatmap();
  rclcpp::shutdown();
  return 0;
}

// #include <string>
// #include <iostream>
// #include <filesystem>
// namespace fs = std::filesystem;
// std::string path = "/path/to/directory";
//     for (const auto & entry : fs::directory_iterator(path))
//         std::cout << entry.path() << std::endl;