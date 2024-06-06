//
// Created by will on 06/06/24.
//


#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "../include/json.hpp"
using json = nlohmann::json;
#include <fstream>
#include <string>

using json = nlohmann::json;

std::vector<visualization_msgs::msg::Marker> generate_markers(const std::string& filename) {
    std::ifstream f(filename);
    json j = json::parse(f);

    std::vector<visualization_msgs::msg::Marker> markers;
    int id = 0;

    for (const auto& sphere : j["spheres"]) {
        geometry_msgs::msg::Point sphere_position;
        sphere_position.x = sphere["x"];
        sphere_position.y = sphere["y"];
        sphere_position.z = sphere["z"];

        int joint_count = 0;
        for (const auto& pose : sphere["poses"]) {
            if (pose.contains("joints") && !pose["joints"].empty()) {
                joint_count++;
            }
        }

        double color_intensity = std::min(1.0, joint_count / 10.0);  // Normalize joint count to a 0-1 scale

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "spheres";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = sphere_position;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = j["radius"];  // Use radius from JSON
        marker.scale.y = j["radius"];
        marker.scale.z = j["radius"];
        marker.color.r = 1.0 - color_intensity;
        marker.color.g = color_intensity;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        markers.push_back(marker);
    }

    return markers;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sphere_visualization");

    std::string filename = "/home/will/master_ik_data.json";
    auto markers = generate_markers(filename);

    auto publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

    rclcpp::Rate rate(1);
    while (rclcpp::ok()) {
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers = markers;
        publisher->publish(marker_array);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
