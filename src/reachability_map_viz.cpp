//
// Created by will on 06/06/24.
//


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "../include/json.hpp"
#include <std_msgs/msg/string.hpp>
using json = nlohmann::json;
#include <fstream>
#include <string>

using json = nlohmann::json;
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class ReachabilityMapVizualisation : public rclcpp::Node
{
public:
  ReachabilityMapVizualisation()
  : Node("reachability_map_vizualisation_node")
  {
    this->declare_parameter("file_name", "/home/will/new_ik_data.json");
    this->file_name = this->get_parameter("file_name").as_string();
    
    // RCLCPP_INFO(node->get_logger(), "msg created");
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&ReachabilityMapVizualisation::timer_callback, this));
  }

    ~ReachabilityMapVizualisation() {}

 void timer_callback()
  {
    auto markers = generate_markers(this->file_name);

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers = markers;
        publisher_->publish(marker_array);
       
    
  }

std::vector<visualization_msgs::msg::Marker> generate_markers(const std::string& filename) {
    std::ifstream f(filename);
    json j = json::parse(f);

    std::vector<visualization_msgs::msg::Marker> markers;
    int id = 0;

    // for (const auto& sphere : j["spheres"]) {
    //     geometry_msgs::msg::Point sphere_position;
    //     sphere_position.x = sphere["x"];
    //     sphere_position.y = sphere["y"];
    //     sphere_position.z = sphere["z"];
    //
    //     int joint_count = 0;
    //     for (const auto& pose : sphere["poses"]) {
    //         if (pose.contains("joints") && !pose["joints"].empty()) {
    //             joint_count++;
    //         }
    //     }

    for (const auto& [sphere_key, sphere_value] : j.items()) {
        if (sphere_key != "radius" && sphere_key != "resolution" && sphere_key != "sphere_sample") {
            // std::istringstream iss(sphere_key);
            // char ignore;
            // double sx, sy, sz;
            // iss >> ignore >> ignore >> ignore >> sx >> ignore >> sy >> ignore >> sz;

            geometry_msgs::msg::Point sphere_position;
            sphere_position.x = sphere_value[0]["x"];
            sphere_position.y = sphere_value[0]["y"];
            sphere_position.z = sphere_value[0]["z"];

            int joint_count = 0;
            for (const auto& [pose_key, pose_array] : sphere_value[0].items()) {
                if (pose_key != "x" && pose_key != "y" && pose_key != "z"){
                    for (const auto& pose : pose_array) {
                        if (pose.contains("joints") && !pose["joints"].empty()) {
                            joint_count++;
                        }
                    }
                }
            }

            double color_intensity = std::min(1.0, joint_count / 50.0);  // Normalize joint count to a 0-1 scale

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
    }

    return markers;
}

private:
std::string file_name;
rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReachabilityMapVizualisation>());

    

    rclcpp::shutdown();
    return 0;
}
