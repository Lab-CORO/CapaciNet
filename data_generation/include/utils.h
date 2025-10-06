//
// Created by will on 06/06/24.
//

#ifndef UTILS_H
#define UTILS_H

#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <fstream>

#include <map>
#include <highfive/H5File.hpp>

#include <boost/multi_array.hpp>
#include <highfive/highfive.hpp>
using namespace std;

    class utils {
    public:
        static double round_to_decimals(double value, int decimals);

        static bool save_poses_to_file(const std::string& filename, const std::vector<geometry_msgs::msg::Pose>& poses);
        static bool load_poses_from_file(const std::string& filename, std::vector<geometry_msgs::msg::Pose>& poses);
        static bool split_data(std::vector<geometry_msgs::msg::Pose>& data,
                                size_t batch_size,
                                std::vector<std::vector<geometry_msgs::msg::Pose>>& batches);
        static bool saveVecToNpz(const std::string& filename, const std::vector<std::array<double, 4>>& data);
        static bool saveToHDF5(const std::map<std::vector<double>, double> &data,
                    const std::vector<std::array<double, 4>> &voxel_grid,
                    float voxel_size,
                    int (&voxel_grid_sizes)[3],
                    double (&voxel_grid_origin)[3],
                    std::shared_ptr<HighFive::File> data_file,
                    int dataset_id);
    };


#endif //UTILS_H
