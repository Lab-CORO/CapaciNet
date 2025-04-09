//
// Created by will on 09/04/24.
//

#ifndef SRC_REACHABILITY_VISUALISATION_H
#define SRC_REACHABILITY_VISUALISATION_H
#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/ColorRGBA.h>
#include "json.hpp"
#include "robot.h"
#include "./master_ik_data.h"
using json = nlohmann::json;

class ReachabilityVisualisation {
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/reachability",  10, true);
public:
    void show_map(std::string filename);
    void setColorFromScore(double score,  std_msgs::ColorRGBA &color);
};


#endif //SRC_REACHABILITY_VISUALISATION_H
