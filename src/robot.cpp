//
// Created by will on 02/04/24.
//

#include "../include/robot.h"

#include <oneapi/tbb/partitioner.h>


Robot::Robot(rclcpp::Node::SharedPtr node) : node_(node) {}
    // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    // robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    // planning_scene_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model);


Robot::~Robot() {}

bool Robot::move_joint(joint &j) {
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    // std::vector<double> joint_group_positions;

    // joint_group_positions.push_back(j.j1);
    // joint_group_positions.push_back(j.j2);
    // joint_group_positions.push_back(j.j3);
    // joint_group_positions.push_back(j.j4);
    // joint_group_positions.push_back(j.j5);
    // joint_group_positions.push_back(j.j6);

    // this->move_group_.setJointValueTarget(joint_group_positions);

    // double planning_timeout = 0.2;
    // move_group_.setPlanningTime(planning_timeout);
    // bool success = (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    return true;
}


bool Robot::get_iks(const std::vector<geometry_msgs::msg::Pose> &poses, std::vector<sensor_msgs::msg::JointState> &joint_states, std::vector<std_msgs::msg::Bool> &joint_states_valid){
    // this method call the service ik with the list of all the poses and return the list of all the joint
    // call the sevice /curobo/ik_poses
    rclcpp::Client<data_generation::srv::Ik>::SharedPtr client = node_->create_client<data_generation::srv::Ik>("/curobo/ik_poses");

    auto request = std::make_shared<data_generation::srv::Ik::Request>();

    request->poses = poses;
    // RCLCPP_INFO(node_->get_logger(), "%f",poses[0].position.x);
    auto result = client->async_send_request(request);


    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        // RCLCPP_INFO(node_->get_logger(), "Service call successful");
        auto res = result.get();
        joint_states = res->joint_states;
        joint_states_valid = res->joint_states_valid;
        // RCLCPP_INFO(node_->get_logger(), "Service call successful");
        return true;

    }else {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed");
        return false;
    }

}

bool Robot::get_all_ik(const std::vector<double> &pose,
                       std::vector<joint> &joints, int &numOfSolns) {
    return true;


    // const moveit::core::RobotStatePtr &current_state = move_group_.getCurrentState();

    // geometry_msgs::msg::Pose goal_pose;
    // goal_pose.position.x = pose[0];
    // goal_pose.position.y = pose[1];
    // goal_pose.position.z = pose[2];
    // goal_pose.orientation.x = pose[3];
    // goal_pose.orientation.y = pose[4];
    // goal_pose.orientation.z = pose[5];
    // goal_pose.orientation.w = pose[6];

    // joint j;
    // std::vector<std::vector<double>> joints_array;

//    get time
//    ros::Time begin = ros::Time::now();
    // bool found_ik = current_state->setFromIK(joint_model_group_, goal_pose);
//    ros::Time end = ros::Time::now();
//    ros::Duration duration = end - begin;
//    RCLCPP_INFO(node->get_logger(),"IK duration: %f", duration.toSec());
//     if (found_ik) {
//         std::vector<double> joint_values;
//         current_state->copyJointGroupPositions(joint_model_group_, joint_values);
// //        get time
// //        begin = ros::Time::now();
//         bool is_collide = checkCollision(joint_values);
// //        end = ros::Time::now();
// //        duration = end - begin;
// //        RCLCPP_INFO(node->get_logger(),"Collision check duration: %f", duration.toSec());
//         if (!is_collide) {

//             // add the joint solution to the vector
//             j.j1 = joint_values[0];
//             j.j2 = joint_values[1];
//             j.j3 = joint_values[2];
//             j.j4 = joint_values[3];
//             j.j5 = joint_values[4];
//             j.j6 = joint_values[5];
// //            get time
// //            begin = ros::Time::now();
//             bool get_path = move_cartesian(goal_pose);
// //            end = ros::Time::now();
// //            duration = end - begin;
// //            RCLCPP_INFO(node->get_logger(),"Move joint duration: %f", duration.toSec());

//             if (get_path) {
//                 joints.push_back(j);
//             } else {
//                 return false;
//             }

//         } else {
//             return false;
//         }
//     } else {
//         return false;
//     }
    return true;
}


bool Robot::checkCollision(const std::vector<double> &joint_positions) {

//    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
//    planning_scene::PlanningScene planning_scene(kinematic_model);

//    const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("manipulator");

//     sleep(5);
//     if (!box_init) {
//         moveit_msgs::CollisionObject collision_object;
//         collision_object.header.frame_id = "base_link";
//         collision_object.id = "obstacle";

//         shape_msgs::SolidPrimitive primitive;
//         primitive.type = primitive.BOX;
//         primitive.dimensions.resize(3);
//         primitive.dimensions[0] = 0.5; // x
//         primitive.dimensions[1] = 0.5; // y
//         primitive.dimensions[2] = 0.5; // z

//         geometry_msgs::msg::Pose box_pose;
//         box_pose.orientation.w = 1.0;
//         box_pose.position.x = 0.5;
//         box_pose.position.y = 0.0;
//         box_pose.position.z = 0;

//         collision_object.primitives.push_back(primitive);
//         collision_object.primitive_poses.push_back(box_pose);
//         collision_object.operation = collision_object.ADD;

//         std::vector<moveit_msgs::CollisionObject> collision_objects;
//         collision_objects.push_back(collision_object);

// //        current_state_.addCollisionObjects(collision_objects);
//         current_state_.applyCollisionObjects(collision_objects);


//         box_init = true;
//     }

//     // Préparation de la requête et du résultat de collision
//     collision_detection::CollisionRequest collision_request;
//     collision_detection::CollisionResult collision_result;
//     collision_request.contacts = true; // Option pour obtenir des détails sur les contacts en cas de collision
//     collision_request.max_contacts = 1000; // Nombre maximal de contacts à rapporter
//     moveit::core::RobotState& current_state_NON = planning_scene_->getCurrentStateNonConst();
//     // Vérification des collisions avec l'environnement
//     planning_scene_->checkCollision(collision_request, collision_result, current_state_NON, planning_scene_->getAllowedCollisionMatrix());

//     if (collision_result.collision) {
//         return true;
//     } else {
        return false;
    // }
}


bool Robot::move_cartesian(geometry_msgs::msg::Pose &pose) {
    // move_group_.setPoseTarget(pose);
    // double planning_timeout = 0.2; // Par exemple, 10 secondes
    // move_group_.setPlanningTime(planning_timeout);
    // bool success = (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    return true;

}