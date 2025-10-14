// The spheres and poses are fused in a single dataset, instead of two datasets for sphere and poses

#include "../include/sphere_discretization.h"
#include "rclcpp/rclcpp.hpp"
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <map>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <sstream>
#include <iostream>
#include <cnpy.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../include/progressbar.hpp"
#include "../include/utils.h"

#include "curobo_msgs/srv/ik_batch.hpp"


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
    auto node = rclcpp::Node::make_shared("create_reachability_map");

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

    float resolution = 0;
    node->declare_parameter("voxel_size", 0.08);
    node->get_parameter("voxel_size", resolution);


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
    float r = 2; // 2 metre ?
     float radius = 0.0001; // size of the sphere that represent the voxel (really small, we want a point)
    octomap::point3d origin = octomap::point3d(0, 0, 0); // This point will be the base of the robot
    octomap::OcTree *tree = sd.generateBoxTree(origin, r, resolution);
    std::vector<octomap::point3d> new_data;
    std::vector<geometry_msgs::msg::Pose> reachability_poses;
    RCLCPP_INFO(node->get_logger(), "Creating the box and discretizing with resolution: %f", resolution);
    int sphere_count = 0;
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
        sphere_count++;
    }
    new_data.reserve(sphere_count);

   

    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
        // new_data.push_back(it.getCoordinate());
        static std::vector<geometry_msgs::msg::Pose> poses_sphere;
        std::vector<double> sphere_coord;
        sd.convertPointToVector(it.getCoordinate(), sphere_coord);
        // create a sphere in the master_ik_data
        geometry_msgs::msg::Point sphere;
        sphere.x = utils::round_to_decimals(sphere_coord[0], 4);
        sphere.y = utils::round_to_decimals(sphere_coord[1], 4);
        sphere.z = utils::round_to_decimals(sphere_coord[2], 4);

        // create poses of spheres exit 50 poses around a sphere pose with a raduis
        sd.make_sphere_poses(sphere, radius, poses_sphere);
        for (std::vector<geometry_msgs::msg::Pose>::iterator it_pose = poses_sphere.begin(); it_pose != poses_sphere.end(); ++it_pose) {
          // the spheres_poses to poses
          reachability_poses.push_back(*it_pose);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Total no of spheres now: %lu", reachability_poses.size());
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

    VectorOfVectors sphere_coord;
    sphere_coord.resize(new_data.size());

    MultiVector pose_col;
    pose_col.reserve(new_data.size() * 50);
    

    // vector of 7d to save with cnpy
    std::vector<geometry_msgs::msg::Pose> poses_vector2save;
    std::map<utils::QuantizedPoint3D, double> map_rm;

    // Split datas to max_batch_size
    std::vector<std::vector<geometry_msgs::msg::Pose>> batches;
    // Set  batch_size as ros param
    int batch_size = 0;
    node->declare_parameter("batch_size", 1000);
    node->get_parameter("batch_size", batch_size);
    utils::split_data(reachability_poses, batch_size, batches);
    RCLCPP_INFO(node->get_logger(), "numbre de batch: %d", batches.size());
    progressbar bar(batches.size());
    int progresse_index = 0;
    for (const auto &batch : batches)
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "Progress: " << generateProgressBar(progresse_index, batches.size()));
        progresse_index ++;
        bar.update();


        // send all the 50 poses to curobo
        std::vector<sensor_msgs::msg::JointState> joint_states;
        std::vector<std_msgs::msg::Bool> joint_states_valid;

        auto request = std::make_shared<curobo_msgs::srv::IkBatch::Request>();

        request->poses = batch;
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
                utils::QuantizedPoint3D pt(batch[j].position.x, batch[j].position.y, batch[j].position.z, resolution);
                // save the pose in vector
                poses_vector2save.push_back(batch[j]);
                map_rm[pt] += 1.0 / 50.0;
            }
        }

    }
    RCLCPP_INFO(node->get_logger(), "Compute finish, saving");
    // save vector to cnpy to the data file in data_generation ros package
    std::stringstream resolution_string;
    resolution_string << resolution;              // appending the float value to the streamclass

    // Save to hdf5
    std::shared_ptr<HighFive::File> data_file_;
    std::string data_file_path;
    data_file_path = ament_index_cpp::get_package_share_directory("data_generation") + "/data/" + "master_ik_data" + resolution_string.str() + ".h5";
    data_file_ = std::make_shared<HighFive::File>(
                data_file_path,
                HighFive::File::ReadWrite | HighFive::File::Create);

    std::vector<std::array<double, 4>> voxel_map = {};
    int rm_size = static_cast<int>(std::round(r / resolution) * 2);
    int voxel_grid_sizes[3] = {rm_size, rm_size, rm_size};
    double voxel_grid_origin[3] = {-r, -r, -r}; 
    RCLCPP_INFO(node->get_logger(), "voxel grid size = %d", rm_size);
    utils::saveToHDF5(map_rm, voxel_map, resolution, voxel_grid_sizes, voxel_grid_origin, data_file_, 0);
    
    data_file_->flush();
    RCLCPP_INFO(node->get_logger(), "HDF5 file saved");
    
    utils::save_poses_to_file(ament_index_cpp::get_package_share_directory("data_generation") + "/data/" +  "/master_ik_data" + resolution_string.str() + ".npz", poses_vector2save);
    // get time
    rclcpp::Time end = node->get_clock()->now();
    rclcpp::Duration duration = end - begin;
    RCLCPP_INFO(node->get_logger(), "Total time taken: %f", duration.seconds());
    RCLCPP_INFO(node->get_logger(), "fini !");
}
