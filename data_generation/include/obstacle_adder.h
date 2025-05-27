#ifndef OBSTACLE_ADDER_HPP
#define OBSTACLE_ADDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "curobo_msgs/srv/add_object.hpp"
#include "curobo_msgs/srv/get_collision_distance.hpp"
#include "curobo_msgs/srv/remove_object.hpp"
#include "curobo_msgs/srv/scene_generator.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <string> 
using namespace std;

#include <random>

class ObstacleAdder : public rclcpp::Node
{
public:
    ObstacleAdder();
    bool add_random_cubes(int nb_object, float max_reach);

private:
    rclcpp::Node::SharedPtr node_;
    // DataGeneration data_generation_;
    rclcpp::Client<curobo_msgs::srv::AddObject>::SharedPtr client_add_obj;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_remove_all_obj;
    rclcpp::Client<curobo_msgs::srv::RemoveObject>::SharedPtr client_remove_obj;
    rclcpp::Client<curobo_msgs::srv::GetCollisionDistance>::SharedPtr client_get_collision_dist;
    rclcpp::Service<curobo_msgs::srv::SceneGenerator>::SharedPtr service_;

    rclcpp::CallbackGroup::SharedPtr client_cb_add_obj, client_cb_rm_obj, client_cb_remove_all, client_cb_collision, server_scene_generator;
    std::mt19937 rng_; // Générateur de nombres aléatoires
    std::uniform_int_distribution<int> num_cubes_dist_; // Distribution pour le nombre de cubes
    std::uniform_real_distribution<double> position_dist_; // Distribution pour les positions
    std::uniform_real_distribution<double> size_dist_; // Distribution pour les dimensions

    

    bool is_in_forbidden_zone();
};

#endif // OBSTACLE_ADDER_HPP
