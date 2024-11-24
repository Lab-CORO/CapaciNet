//
// Created by will on 04/04/24.
//

#include "../include/data_generator.h"

DataGenerator::DataGenerator() : Node("data_generator"), robot(this->shared_from_this())
{
    // robot = Robot(this->shared_from_this());
    this->service_ = this->create_service<curobo_msgs::srv::GenerateRM>(
        "generate_rm", std::bind(&DataGenerator::callback_generate_rm, this, std::placeholders::_1, std::placeholders::_2));
}

DataGenerator::~DataGenerator() {}

void DataGenerator::callback_generate_rm(
    const std::shared_ptr<curobo_msgs::srv::GenerateRM::Request> request,
    std::shared_ptr<curobo_msgs::srv::GenerateRM::Response> response)
{
    float resolution = request->resolution;
    std::vector<geometry_msgs::msg::Pose> data;

    std::stringstream resolution_string;
    resolution_string << resolution;              // appending the float value to the streamclass
    std::string result = resolution_string.str(); // converting the float value to string

    utils::load_poses_from_file(ament_index_cpp::get_package_share_directory("data_generation") + "/data" + "/master_ik_data" + result + ".npz", data);

    // split data into batches
    int batch_size = request->batch_size;
    std::vector<std::vector<geometry_msgs::msg::Pose>> batches;
    utils::split_data(data, batch_size, batches);

    // create a master ik data object
    MasterIkData ik_data;

    // iterate all batches
    for (const auto &batch : batches)
    {
        // send the batch to robot ik
        std::vector<sensor_msgs::msg::JointState> joint_states;
        std::vector<std_msgs::msg::Bool> joint_states_valid;
        robot.get_iks(batch, joint_states, joint_states_valid);
        int nb_valide = 0;
        for (size_t i = 0; i < joint_states.size(); i++)
        {
            // from ik_data get the sphere with key x, y, z
            if (joint_states_valid[i].data)
            {
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
                nb_valide++;
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
    // rnd name for file

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    // std::string date = std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d_%m_%Y_%H_%M_%S");
    auto date_str = oss.str();
    ik_data.write_data(ament_index_cpp::get_package_share_directory("data_generation") + "/data/" + date_str + "_" + result + ".json");


    response->success = true;
    response->message = "Reachability map generated successfully.";
       
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<DataGenerator>());
    rclcpp::shutdown();
    return 0;
}

/**
// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<DataGenerator>());
//     rclcpp::shutdown();
//     return 0;
// }

// #include <iostream>
// #include <fstream>
// #include <ctime>

// #include "rclcpp/rclcpp.hpp"

// #include "../include/robot.h"
// #include "../include/master_ik_data.h"
// #include "../include/utils.h"
// #include "curobo_msgs/srv/generate_rm.hpp"

// #include <ament_index_cpp/get_package_share_directory.hpp>

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("workspace");
//     // create robot
//     Robot robot(node);
//     // load data from vector
//     float resolution = 0.5;
//     std::vector<geometry_msgs::msg::Pose> data;

//     std::stringstream resolution_string;
//     resolution_string << resolution;              // appending the float value to the streamclass
//     std::string result = resolution_string.str(); // converting the float value to string

//     utils::load_poses_from_file(ament_index_cpp::get_package_share_directory("data_generation") + "/data" + "/master_ik_data" + result + ".npz", data);

//     // split data into batches
//     size_t batch_size = 1500;
//     std::vector<std::vector<geometry_msgs::msg::Pose>> batches;
//     utils::split_data(data, batch_size, batches);

//     // create a master ik data object
//     MasterIkData ik_data;

//     // iterate all batches
//     for (const auto &batch : batches)
//     {
//         // send the batch to robot ik
//         std::vector<sensor_msgs::msg::JointState> joint_states;
//         std::vector<std_msgs::msg::Bool> joint_states_valid;
//         robot.get_iks(batch, joint_states, joint_states_valid);
//         int nb_valide = 0;
//         for (size_t i = 0; i < joint_states.size(); i++)
//         {
//             // from ik_data get the sphere with key x, y, z
//             if (joint_states_valid[i].data)
//             {
//                 // create a pose in the sphere
//                 PoseOnSphere p;
//                 p.x = utils::round_to_decimals(batch[i].position.x, 3);
//                 p.y = utils::round_to_decimals(batch[i].position.y, 3);
//                 p.z = utils::round_to_decimals(batch[i].position.z, 3);
//                 p.theta_x = utils::round_to_decimals(batch[i].orientation.x, 6);
//                 p.theta_y = utils::round_to_decimals(batch[i].orientation.y, 6);
//                 p.theta_z = utils::round_to_decimals(batch[i].orientation.z, 6);
//                 p.theta_w = utils::round_to_decimals(batch[i].orientation.w, 6);
//                 // if joint[i] is not empty add the joints to the pose
//                 nb_valide++;
//                 // create a joint
//                 joint join;
//                 join.j1 = joint_states[i].position[0];
//                 join.j2 = joint_states[i].position[1];
//                 join.j3 = joint_states[i].position[2];
//                 join.j4 = joint_states[i].position[3];
//                 join.j5 = joint_states[i].position[4];
//                 join.j6 = joint_states[i].position[5];
//                 // add the joint to the pose
//                 std::vector<joint> joints = {join};
//                 p.add(joints);
//                 // add the pose to the sphere
//                 ik_data.update_sphere(batch[i].position.x, batch[i].position.y, batch[i].position.z, p);
//             }
//         }
//     }
//     // rnd name for file

//     auto t = std::time(nullptr);
//     auto tm = *std::localtime(&t);
//     // std::string date = std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
//     std::ostringstream oss;
//     oss << std::put_time(&tm, "%d_%m_%Y_%H_%M_%S");
//     auto date_str = oss.str();
//     ik_data.write_data(ament_index_cpp::get_package_share_directory("data_generation") + "/data/" + date_str + "_" + result + ".json");
// }
*/