// The spheres and poses are fused in a single dataset, instead of two datasets for sphere and poses

#include "../include/sphere_discretization.h"
#include "rclcpp/rclcpp.hpp"
// #include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_array.hpp>

#include <map>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <sstream>
#include <iostream>
#include <cnpy.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../include/progressbar.hpp"

#include "../include/master_ik_data.h"
#include "../include/robot.h"
#include "../include/utils.h"

typedef std::multimap<const std::vector<double> *, const std::vector<double> *> MultiMapPtr;
typedef std::map<const std::vector<double> *, double> MapVecDoublePtr;
typedef std::multimap<std::vector<double>, std::vector<double>> MultiMap;
typedef std::map<std::vector<double>, double> MapVecDouble;
typedef std::vector<std::vector<double>> VectorOfVectors;
struct stat st;
typedef std::vector<std::pair<std::vector<double>, const std::vector<double> *>> MultiVector;

using namespace std::chrono_literals;

std::string generateProgressBar(int current, int total, int bar_width = 50)
{
  // Ensure we don't have a division by zero
  if (total <= 0) {
    return "[Error: total <= 0]";
  }

  double progress = static_cast<double>(current) / static_cast<double>(total);
  // Clamp progress to [0, 1]
  if (progress < 0.0) progress = 0.0;
  if (progress > 1.0) progress = 1.0;

  int pos = static_cast<int>(bar_width * progress);

  std::ostringstream bar;
  bar << "[";
  for (int i = 0; i < bar_width; ++i) {
    if (i < pos) {
      bar << "=";
    } else if (i == pos) {
      bar << ">";
    } else {
      bar << " ";
    }
  }
  bar << "] " << static_cast<int>(progress * 100.0) << " %";

  return bar.str();
}

bool isFloat(std::string s)
{
    std::istringstream iss(s);
    float dummy;
    iss >> std::noskipws >> dummy;
    return iss && iss.eof(); // Result converted to bool
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("workspace");

    rclcpp::Client<curobo_msgs::srv::IkBatch>::SharedPtr client_ik = node->create_client<curobo_msgs::srv::IkBatch>("/curobo_ik/ik_batch_poses", rmw_qos_profile_services_default);
    // wait for the service up
    while (!client_ik->wait_for_service(5s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    bool debug = false;
    rclcpp::Time startit = node->get_clock()->now();

    float voxel_size = 0;
    node->declare_parameter("voxel_size", 0.08);
    node->get_parameter("voxel_size", voxel_size);
    float resolution = voxel_size;

    int count = 0;
    //    get time
    rclcpp::Time begin = node->get_clock()->now();

    unsigned char max_depth = 16;
    unsigned char minDepth = 0;

    // A box of radius 1 is created. It will be the size of the robot+1.5. Then the box is discretized by voxels of
    // specified resolution

    // TODO resolution will be user argument
    // The center of every voxels are stored in a vector

    sphere_discretization::SphereDiscretization sd;
    float r = 2;
    octomap::point3d origin = octomap::point3d(0, 0, 0); // This point will be the base of the robot
    octomap::OcTree *tree = sd.generateBoxTree(origin, r, resolution);
    std::vector<octomap::point3d> new_data;
    RCLCPP_INFO(node->get_logger(), "Creating the box and discretizing with resolution: %f", resolution);
    int sphere_count = 0;
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
        sphere_count++;
    }
    new_data.reserve(sphere_count);
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
        new_data.push_back(it.getCoordinate());
    }

    RCLCPP_INFO(node->get_logger(), "Total no of spheres now: %lu", new_data.size());
    RCLCPP_INFO(node->get_logger(),
                "Please hold ON. Spheres are discretized and all of the poses are checked for Ik solutions. May take some "
                "time");

    // A sphere is created in every voxel. The sphere may be created by default or other techniques.
    // TODO Other techniques need to be modified. the user can specifiy which technique they want to use
    // TODO The sphere discretization parameter and rotation of every poses will be taken as argument. If the final
    // joints can rotate (0, 2pi) we dont need to rotate the poses.
    // Every discretized points on spheres are converted to pose and all the poses are saved in a multimap with their
    // corresponding sphere centers
    // If the resolution is 0.01 the programs not responds
    // TODO seperate raduise and resolutiion
    float radius = 0.0001;

    VectorOfVectors sphere_coord;
    sphere_coord.resize(new_data.size());

    MultiVector pose_col;
    pose_col.reserve(new_data.size() * 50);
    MasterIkData data_ik;
    progressbar bar(new_data.size());

    // vector of 7d to save with cnpy
    std::vector<geometry_msgs::msg::Pose> poses_vector2save;

    for (int i = 0; i < new_data.size(); i++)
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "Progress: " << generateProgressBar(i, new_data.size()));
        // bar.update();
        static std::vector<geometry_msgs::msg::Pose> poses;
        sd.convertPointToVector(new_data[i], sphere_coord[i]);
        // create a sphere in the master_ik_data
        Sphere sphere;
        sphere.x = utils::round_to_decimals(sphere_coord[i][0], 4);
        sphere.y = utils::round_to_decimals(sphere_coord[i][1], 4);
        sphere.z = utils::round_to_decimals(sphere_coord[i][2], 4);

        // create poses of spheres exit 50 poses around a sphere pose with a raduis
        sd.make_sphere_poses(sphere, radius, poses);
        // number of point accepted for curobo

        // send all the 50 poses to curobo
        std::vector<sensor_msgs::msg::JointState> joint_states;
        std::vector<std_msgs::msg::Bool> joint_states_valid;

        auto request = std::make_shared<curobo_msgs::srv::IkBatch::Request>();

        request->poses = poses;
        auto result_future = client_ik->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto res = result_future.get();
            joint_states = res->joint_states;
            joint_states_valid = res->joint_states_valid;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Service call failed");
            return false;
        }

        for (int j = 0; j < joint_states.size(); j++)
        {
            if (joint_states_valid[j].data)
            {
                // save the pose in vector

                poses_vector2save.push_back(poses[j]);

                // // create a pose in the sphere
                // PoseOnSphere p;
                // p.x = sphere.x;
                // p.y = sphere.y;
                // p.z = sphere.z;
                // p.theta_x = utils::round_to_decimals(poses[j].orientation.x, 6);
                // p.theta_y = utils::round_to_decimals(poses[j].orientation.y, 6);
                // p.theta_z = utils::round_to_decimals(poses[j].orientation.z, 6);
                // p.theta_w = utils::round_to_decimals(poses[j].orientation.w, 6);
                // // if joint[i] is not empty add the joints to the pose

                // // create a joint
                // joint join;
                // join.j1 = joint_states[j].position[0];
                // join.j2 = joint_states[j].position[1];
                // join.j3 = joint_states[j].position[2];
                // join.j4 = joint_states[j].position[3];
                // join.j5 = joint_states[j].position[4];
                // join.j6 = joint_states[j].position[5];
                // // add the joint to the pose
                // std::vector<joint> joints = {join};
                // p.add(joints);
                // // add the pose to the sphere
                // sphere.add(p);
            }
        }
        // //  add the sphere to the master_ik
        // if (sphere.has_points())
        // {
        //     data_ik.add(sphere);
        // }
    }

    // save vector to cnpy to the data file in data_generation ros package
    std::stringstream resolution_string;
    resolution_string << resolution;              // appending the float value to the streamclass
    std::string result = resolution_string.str(); // converting the float value to string
    utils::save_poses_to_file(std::string("/home/ros2_ws/src/") +  "/master_ik_data" + result + ".npz", poses_vector2save);
    // data_ik.write_data(std::string("/home/ros2_ws/src/capacitynet")+ "/data"  + "/master_ik_data" + result + ".json");
        // ament_index_cpp::get_package_share_directory("data_generation") + "/data"  + "/master_ik_data" + result + ".json");
    // get time
    rclcpp::Time end = node->get_clock()->now();
    rclcpp::Duration duration = end - begin;
    RCLCPP_INFO(node->get_logger(), "Total time taken: %f", duration.seconds());
    RCLCPP_INFO(node->get_logger(), "fini !");
}
