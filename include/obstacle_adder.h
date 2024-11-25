#ifndef OBSTACLE_ADDER_HPP
#define OBSTACLE_ADDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "curobo_msgs/srv/add_object.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include <random>

class ObstacleAdder : public rclcpp::Node
{
public:
    ObstacleAdder();
    void add_random_cubes();

private:
    rclcpp::Client<curobo_msgs::srv::AddObject>::SharedPtr client_;
    std::mt19937 rng_; // Générateur de nombres aléatoires
    std::uniform_int_distribution<int> num_cubes_dist_; // Distribution pour le nombre de cubes
    std::uniform_real_distribution<double> position_dist_; // Distribution pour les positions
    std::uniform_real_distribution<double> size_dist_; // Distribution pour les dimensions

    /// @brief Verify the cube to add is not in the forbidden zone
    /// @param x the x position of the cube
    /// @param y the y position of the cube
    /// @param z the z position of the cube
    /// @param size_x the size of the cube in x
    /// @param size_y the size of the cube in y
    /// @param size_z 
    /// @param x_min the x position of the forbidden zone
    /// @param x_max the x position of the forbidden zone
    /// @param y_min the y position of the forbidden zone
    /// @param y_max the y position of the forbidden zone
    /// @param z_min the z position of the forbidden zone
    /// @param z_max the z position of the forbidden zone
    /// @return true if the cube is in the forbidden zone, false otherwise
    bool is_in_forbidden_zone(double x, double y, double z, 
                                            double size_x, double size_y, double size_z, 
                                            double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
};

#endif // OBSTACLE_ADDER_HPP
