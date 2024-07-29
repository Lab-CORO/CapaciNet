//
// Created by will on 02/04/24.
//

#ifndef SRC_ROBOT_H
#define SRC_ROBOT_H


#include <Eigen/Dense>
#include "capacinet_msg/srv/ik.hpp"
#include "master_ik_data.h"

class Robot {

public:
    Robot(rclcpp::Node::SharedPtr node);
    ~Robot();
    bool get_iks(const std::vector<geometry_msgs::msg::Pose> &poses, std::vector<sensor_msgs::msg::JointState> &joint_states, std::vector<std_msgs::msg::Bool> &joint_states_valid);
    bool get_all_ik( const std::vector<double> &pose, std::vector<joint> &joints, int &numOfSolns);
    bool move_joint(joint &j);
    bool move_cartesian(geometry_msgs::msg::Pose &pose);
    bool checkCollision(const std::vector<double>& joint_positions);

private:
    // ros::NodeHandle node_handle;
    // moveit::planning_interface::MoveGroupInterface move_group_;
    // const robot_state::JointModelGroup* joint_model_group_;
    // planning_scene::PlanningScenePtr planning_scene_;
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
//    moveit::core::RobotState &current_state_NonConst;
    // moveit::planning_interface::PlanningSceneInterface current_state_;
    bool box_init{};
    rclcpp::Node::SharedPtr node_;



};


#endif //SRC_ROBOT_H
