//
// Created by will on 04/04/24.
//


#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"

#include "../include/robot.h"
#include "../include/master_ik_data.h"
#include "../include/utils.h"


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("workspace");
    // create robot
    Robot robot(node);
    // load data from vector

    std::vector<geometry_msgs::msg::Pose> data;
    utils::load_poses_from_file("data.bin", data);

    // split data into batches
    size_t batch_size = 1000;
    std::vector<std::vector<geometry_msgs::msg::Pose>> batches;
    utils::split_data(data,  batch_size, batches);

    // create a master ik data object
    MasterIkData ik_data;



    // iterate all batches
    for (const auto& batch : batches) {
        // send the batch to robot ik
        std::vector<sensor_msgs::msg::JointState>  joint_states;
        std::vector<std_msgs::msg::Bool> joint_states_valid;
        robot.get_iks(batch, joint_states, joint_states_valid);

        for (size_t i = 0; i < joint_states.size(); i++) {
            // add the joint states to the master ik data object

        }


    }
}




