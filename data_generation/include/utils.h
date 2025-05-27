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


    class utils {
    public:
        static double round_to_decimals(double value, int decimals);

        static bool save_poses_to_file(const std::string& filename, const std::vector<geometry_msgs::msg::Pose>& poses);
        static bool load_poses_from_file(const std::string& filename, std::vector<geometry_msgs::msg::Pose>& poses);
        static bool split_data(const std::vector<geometry_msgs::msg::Pose>& data,
                                        size_t batch_size,
                                        std::vector<std::vector<geometry_msgs::msg::Pose>>& batches);
        static bool saveVecToNpz(const std::string& filename, const std::vector<std::array<double, 4>>& data);
    };


#endif //UTILS_H
