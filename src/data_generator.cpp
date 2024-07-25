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
    utils::load_poses_from_file("/home/will/master_ik_data.npy", data);

    // split data into batches
    size_t batch_size = 1500;
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
        int nb_valide = 0;
        for (size_t i = 0; i < joint_states.size(); i++) {
            // from ik_data get the sphere with key x, y, z
            if (joint_states_valid[i].data) {
                // create a pose in the sphere
                PoseOnSphere p;
                p.x = utils::round_to_decimals(batch[i].position.x, 3);
                p.y = utils::round_to_decimals(batch[i].position.y, 3);
                p.z = utils::round_to_decimals(batch[i].position.z, 3);
                p.theta_x = utils::round_to_decimals(batch[i].orientation.x, 6);
                p.theta_y = utils::round_to_decimals(batch[i].orientation.y, 6);
                p.theta_z = utils::round_to_decimals(batch[i].orientation.z, 6);
                p.theta_w = utils::round_to_decimals(batch[i].orientation.w, 6);
                // if joint[i] is not empty add the joints to the pose
                nb_valide ++;
                // create a joint
                joint join;
                join.j1 = joint_states[i].position[0];
                join.j2 = joint_states[i].position[1];
                join.j3 = joint_states[i].position[2];
                join.j4 = joint_states[i].position[3];
                join.j5 = joint_states[i].position[4];
                join.j6 = joint_states[i].position[5];
                // add the joint to the pose
                std::vector<joint> joints = {join};
                p.add(joints);
                // add the pose to the sphere
                ik_data.update_sphere(batch[i].position.x, batch[i].position.y, batch[i].position.z, p);
            }
        }


    }
    //rnd name for file

    ik_data.write_data("/home/will/new_ik_data_test.json");

}




