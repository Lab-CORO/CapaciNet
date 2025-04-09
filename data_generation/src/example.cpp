#include "rclcpp/rclcpp.hpp"
// PCL specific includes
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//ros::Publisher pub;
//
//void
//cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
//{
//    // Create a container for the data.
//    sensor_msgs::PointCloud2 output;
//
//    // Do data processing here...
//    output = *input;
//
//    // Publish the data.
//    pub.publish (output);
//}
//
//int
//main (int argc, char** argv)
//{
//    // Initialize ROS
//    ros::init (argc, argv, "my_pcl_tutorial");
//    ros::NodeHandle nh;
//
//    // Create a ROS subscriber for the input point cloud
//    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
//
//    // Create a ROS publisher for the output point cloud
//    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
//
//    // Spin
//    ros::spin ();
//}

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>


void addObstacle(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link"; // Assurez-vous que cela correspond à votre robot
    collision_object.id = "obstacle";

    // Définir la forme et la taille de l'obstacle
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.5; // x
    primitive.dimensions[1] = 0.5; // y
    primitive.dimensions[2] = 0.5; // z

    // Définir la position et l'orientation de l'obstacle
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.applyCollisionObjects(collision_objects);
}


bool checkCollision(moveit::planning_interface::MoveGroupInterface& move_group, ros::NodeHandle& nh, std::vector<double> &joint_positions)
{
    moveit_msgs::GetStateValidity srv;


    srv.request.group_name = move_group.getName();
    move_group.getCurrentState()->update();
    move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), srv.request.robot_state.joint_state.position);
    //    change joint position dans srv
    srv.request.robot_state.joint_state.position = joint_positions;
    ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetStateValidity>("check_state_validity");
    if (client.call(srv))
    {
        return !srv.response.valid;
    }
    else
    {
        ROS_ERROR("Failed to call service check_state_validity");
        return true; // Assume there is a collision if the service call failed
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "check_collision_example");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Ajouter un obstacle
    addObstacle(planning_scene_interface);

    // Vérifier la collision
    std::vector<double> joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool in_collision = checkCollision(move_group, nh, joint_positions);
    if (in_collision)
    {
        RCLCPP_INFO(node->get_logger(),"La configuration est en collision.");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(),"La configuration n'est pas en collision.");
    }

    ros::shutdown();
    return 0;
}
