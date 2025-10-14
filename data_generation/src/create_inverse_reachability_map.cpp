#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>

#include "../include/sphere_discretization.h"
#include "../include/utils.h"
// #include "map_creator/msg/work_space.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <ctime>
#include <string>
#include <time.h>
#include <filesystem>


#include <highfive/H5File.hpp>

#include <highfive/highfive.hpp>

using namespace HighFive;

// --- MultiMapPtr : mapping entre points de base et poses accessibles
typedef std::multimap<const geometry_msgs::msg::Point*, const geometry_msgs::msg::Pose*> MultiMapPosePtr;

// --- MapVecDoublePtr : associe une position (clé) à un score
typedef std::map<const geometry_msgs::msg::Point*, double> MapPointDoublePtr;


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("inverse_workspace");
  
  time_t startit, finish;
  time(&startit);
  
  std::string file;
  std::string pkg_share_dir;


  rclcpp::Rate loop_rate(10);

  int count = 0;
  while (rclcpp::ok())
  {
    std::vector<geometry_msgs::msg::Pose> pose_col_filter;
    MapPointDoublePtr sphere_col;

    utils::load_poses_from_file("/home/ros2_ws/src/CapaciNet/data_generation/data/master_ik_data0.5.npz", pose_col_filter);

    // Create Inverse Reachability map
    unsigned char max_depth = 16;
    unsigned char minDepth = 0;
    float size_of_box = 1.5;
    float resolution = 0.3;
    sphere_discretization::SphereDiscretization sd;

    octomap::point3d origin = octomap::point3d(0, 0, 0);
    octomap::OcTree *tree = sd.generateBoxTree(origin, size_of_box, resolution);
    std::vector<octomap::point3d> new_data;

    std::vector<geometry_msgs::msg::Pose> pose;
    geometry_msgs::msg::Point orgine_pt;
    orgine_pt.x = 0;
    orgine_pt.y = 0;
    orgine_pt.z = 0;
    sd.make_sphere_poses(orgine_pt, resolution, pose);

    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); 
         it != end; ++it)
    {
      new_data.push_back(it.getCoordinate());
    }

    RCLCPP_INFO(node->get_logger(), "Number of poses in RM: %lu", pose_col_filter.size());
    RCLCPP_INFO(node->get_logger(), "Number of voxels: %lu", new_data.size());

    // Transform all poses to transformation matrices
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::multimap<std::vector<float>, std::vector<float>> trns_col;
    
    for (std::vector<geometry_msgs::msg::Pose>::iterator it = pose_col_filter.begin(); it != pose_col_filter.end(); ++it)
    {
      tf2::Vector3 vec(it->position.x, it->position.y, it->position.z);
      tf2::Quaternion quat(it->orientation.x, it->orientation.y, it->orientation.z, it->orientation.w);
      tf2::Transform trns;
      trns.setOrigin(vec);
      trns.setRotation(quat);
      
      tf2::Transform trns_inv;
      trns_inv = trns.inverse();

      tf2::Vector3 inv_trans_vec;
      tf2::Quaternion inv_trans_quat;
      inv_trans_vec = trns_inv.getOrigin();
      inv_trans_quat = trns_inv.getRotation();
      inv_trans_quat.normalize();

      std::vector<float> position;
      position.push_back(inv_trans_vec[0]);
      position.push_back(inv_trans_vec[1]);
      position.push_back(inv_trans_vec[2]);
      
      std::vector<float> orientation;
      orientation.push_back(inv_trans_quat[0]);
      orientation.push_back(inv_trans_quat[1]);
      orientation.push_back(inv_trans_quat[2]);
      orientation.push_back(inv_trans_quat[3]);

      trns_col.insert(std::pair<std::vector<float>, std::vector<float>>(position, orientation));

      pcl::PointXYZ point;
      point.x = inv_trans_vec[0];
      point.y = inv_trans_vec[1];
      point.z = inv_trans_vec[2];
      cloud->push_back(point);
    }
    MultiMapPosePtr base_trns_col;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    for (size_t i = 0; i < new_data.size(); i++)
    {
      pcl::PointXYZ search_point;
      search_point.x = new_data[i].x();
      search_point.y = new_data[i].y();
      search_point.z = new_data[i].z();

      // Neighbors within voxel search
      std::vector<int> point_idx_vec;
      octree.voxelSearch(search_point, point_idx_vec);
      if (point_idx_vec.size() > 0)
      {
        geometry_msgs::msg::Point* base_sphere = new geometry_msgs::msg::Point(); 
        base_sphere->x = search_point.x;
        base_sphere->y = search_point.y;
        base_sphere->z = search_point.z;
        for (size_t j = 0; j < point_idx_vec.size(); ++j)
        {
          std::vector<float> base_pos;
          base_pos.push_back(cloud->points[point_idx_vec[j]].x);
          base_pos.push_back(cloud->points[point_idx_vec[j]].y);
          base_pos.push_back(cloud->points[point_idx_vec[j]].z);

          for (auto it1 = trns_col.lower_bound(base_pos); it1 != trns_col.upper_bound(base_pos); ++it1)
          {
            geometry_msgs::msg::Pose* base_pose = new geometry_msgs::msg::Pose();
            base_pose->position.x = (base_pos[0]);
            base_pose->position.y = (base_pos[1]);
            base_pose->position.z = (base_pos[2]);
            base_pose->orientation.x =(it1->second[0]);
            base_pose->orientation.y = (it1->second[1]);
            base_pose->orientation.z = (it1->second[2]);
            base_pose->orientation.w = (it1->second[3]);

            base_trns_col.insert(std::make_pair(base_sphere, base_pose));
          }
        }
      }
    }
    std::map<utils::QuantizedPoint3D, double> IRM;
    for (MultiMapPosePtr::iterator it = base_trns_col.begin(); it != base_trns_col.end(); ++it)
    {      
      utils::QuantizedPoint3D sphere_coord(it->first->x, it->first->y, it->first->z, resolution);
      float d = (float(base_trns_col.count(it->first)) / pose.size()) * 100;
      IRM[sphere_coord] = d;
    }

    RCLCPP_INFO(node->get_logger(), "Number of Spheres in RM: %lu", sphere_col.size());

    RCLCPP_INFO(node->get_logger(), 
      "All the poses have been processed. Now saving data to an inverse Reachability Map.");

    std::vector<std::array<double, 4>> voxel_grid = {};
    std::string data_file_path = "/home/ros2_ws/src/CapaciNet/data_generation/data/IRM.h5";
    std::shared_ptr<HighFive::File> data_file_ = std::make_shared<HighFive::File>(
                data_file_path,
                HighFive::File::ReadWrite | HighFive::File::Create);
    int voxel_grid_sizes[3] = {14, 14, 14};
    double voxel_grid_origin[3] = {-1.5, -1.5, -1.5};
    // load IRM
    utils::saveToHDF5(IRM,
            voxel_grid,
            resolution,
            voxel_grid_sizes,
            voxel_grid_origin,
            data_file_,
            0
            );
    time(&finish);
    double dif = difftime(finish, startit);
    RCLCPP_INFO(node->get_logger(), "Elapsed time is %.2lf seconds.", dif);
    RCLCPP_INFO(node->get_logger(), "Completed");
    
    rclcpp::spin_some(node);
    return 0;
  }

  rclcpp::shutdown();
  return 0;
}