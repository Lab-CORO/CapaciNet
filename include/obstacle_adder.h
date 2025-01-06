#ifndef OBSTACLE_ADDER_HPP
#define OBSTACLE_ADDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "curobo_msgs/srv/add_object.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <random>

class ObstacleAdder : public rclcpp::Node
{
public:
    ObstacleAdder();
    void add_random_cubes();

private:
    rclcpp::Node::SharedPtr node_;
    // DataGeneration data_generation_;
    rclcpp::Client<curobo_msgs::srv::AddObject>::SharedPtr client_add_obj;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_remove_obj;
    std::mt19937 rng_; // Générateur de nombres aléatoires
    std::uniform_int_distribution<int> num_cubes_dist_; // Distribution pour le nombre de cubes
    std::uniform_real_distribution<double> position_dist_; // Distribution pour les positions
    std::uniform_real_distribution<double> size_dist_; // Distribution pour les dimensions


    bool is_in_forbidden_zone(double x, double y, double z, 
                                            double size_x, double size_y, double size_z, 
                                            double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
};

#endif // OBSTACLE_ADDER_HPP
