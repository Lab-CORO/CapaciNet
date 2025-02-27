//
// Created by will on 04/04/24.
//

#ifndef SRC_DATA_GENERATOR_H
#define SRC_DATA_GENERATOR_H

#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "json.hpp"
#include "robot.h"
#include "./master_ik_data.h"
#include "../include/robot.h"
#include "../include/master_ik_data.h"
#include "../include/utils.h"
#include "curobo_msgs/srv/generate_rm.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ctime>

using json = nlohmann::json;

class DataGenerator : public rclcpp::Node
{

public:
    DataGenerator(rclcpp::Node::SharedPtr node);
    ~DataGenerator();
    
    

private:
    //    json file
    std::string filename;
    json json_data;
    MasterIkData ik_data;
    MasterIkData ik_data_result;
    rclcpp::Node::SharedPtr node_;
    // rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    // rclcpp::Node node;
    // rclcpp::Service<curobo_msgs::srv::GenerateRM>::SharedPtr service_;
    Robot robot;

    void callback_generate_rm(float resolutiion, int batch_size);
};

#endif // SRC_DATA_GENERATOR_H
