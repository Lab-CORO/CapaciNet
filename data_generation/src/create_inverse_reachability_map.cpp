#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>

#include <map_creator/sphere_discretization.h>
#include <map_creator/kinematics.h>
#include <map_creator/hdf5_dataset.h>
#include "map_creator/msg/work_space.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <ctime>
#include <string>
#include <time.h>
#include <filesystem>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("inverse_workspace");
  
  time_t startit, finish;
  time(&startit);
  
  kinematics::Kinematics k(node);
  std::string file;
  std::string pkg_share_dir;
  
  try {
    pkg_share_dir = ament_index_cpp::get_package_share_directory("map_creator");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get package directory: %s", e.what());
    return 1;
  }
  
  std::string path = pkg_share_dir + "/maps/";
  std::string filename;
  const char *input_FILE;

  if (argc < 2)
  {
    RCLCPP_ERROR(node->get_logger(), 
      "Please provide the name of the reachability map. If you have not created it yet, "
      "Please create the map by running the create reachability map node in map_creator package");
    return 0;
  }
  else if (argc == 2)
  {
    RCLCPP_INFO(node->get_logger(), "Creating map with default name.");
    input_FILE = argv[1];
    
    if (!std::filesystem::exists(input_FILE))
    {
      RCLCPP_ERROR(node->get_logger(), "Input file does not exist");
      return false;
    }
    else
    {
      float res;
      hdf5_dataset::Hdf5Dataset h5_res(argv[1]);
      h5_res.open();
      h5_res.h5ToResolution(res);
      h5_res.close();
      
      file = str(boost::format("%s_r%d_Inv_reachability.h5") % k.getRobotName() % res);
      filename = path + file;
    }
  }
  else if (argc == 3)
  {
    input_FILE = argv[1];
    std::string str(argv[2]);
    
    if (!std::filesystem::exists(input_FILE))
    {
      RCLCPP_ERROR(node->get_logger(), "Input file does not exist");
      return false;
    }
    else
    {
      if (std::strchr(str.c_str(), '/'))
      {
        filename = argv[2];
      }
      else
        filename = path + str;
    }
  }

  rclcpp::Rate loop_rate(10);

  int count = 0;
  while (rclcpp::ok())
  {
    MultiMapPtr pose_col_filter;
    MapVecDoublePtr sphere_col;
    float res;

    hdf5_dataset::Hdf5Dataset h5file(input_FILE);
    h5file.open();
    h5file.loadMapsFromDataset(pose_col_filter, sphere_col, res);

    // Create Inverse Reachability map
    unsigned char max_depth = 16;
    unsigned char minDepth = 0;
    float size_of_box = 1.5;
    float resolution = res;
    sphere_discretization::SphereDiscretization sd;

    octomap::point3d origin = octomap::point3d(0, 0, 0);
    octomap::OcTree *tree = sd.generateBoxTree(origin, size_of_box, resolution);
    std::vector<octomap::point3d> new_data;

    std::vector<geometry_msgs::msg::Pose> pose;
    sd.make_sphere_poses(origin, resolution, pose);

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
    
    for (MultiMapPtr::iterator it = pose_col_filter.begin(); it != pose_col_filter.end(); ++it)
    {
      tf2::Vector3 vec((*it->second)[0], (*it->second)[1], (*it->second)[2]);
      tf2::Quaternion quat((*it->second)[3], (*it->second)[4], (*it->second)[5], (*it->second)[6]);
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

    MultiMapPtr base_trns_col;
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
        std::vector<double>* base_sphere = new std::vector<double>();
        base_sphere->push_back(search_point.x);
        base_sphere->push_back(search_point.y);
        base_sphere->push_back(search_point.z);

        for (size_t j = 0; j < point_idx_vec.size(); ++j)
        {
          std::vector<float> base_pos;
          base_pos.push_back(cloud->points[point_idx_vec[j]].x);
          base_pos.push_back(cloud->points[point_idx_vec[j]].y);
          base_pos.push_back(cloud->points[point_idx_vec[j]].z);

          for (auto it1 = trns_col.lower_bound(base_pos); it1 != trns_col.upper_bound(base_pos); ++it1)
          {
            std::vector<double>* base_pose = new std::vector<double>();
            base_pose->push_back(base_pos[0]);
            base_pose->push_back(base_pos[1]);
            base_pose->push_back(base_pos[2]);
            base_pose->push_back(it1->second[0]);
            base_pose->push_back(it1->second[1]);
            base_pose->push_back(it1->second[2]);
            base_pose->push_back(it1->second[3]);

            base_trns_col.insert(std::make_pair(base_sphere, base_pose));
          }
        }
      }
    }

    MapVecDoublePtr sphere_color;
    for (MultiMapPtr::iterator it = base_trns_col.begin(); it != base_trns_col.end(); ++it)
    {
      const std::vector<double>* sphere_coord = it->first;
      float d = (float(base_trns_col.count(sphere_coord)) / pose.size()) * 100;
      sphere_color.insert(std::make_pair(it->first, double(d)));
    }

    RCLCPP_INFO(node->get_logger(), "Number of Spheres in RM: %lu", sphere_col.size());
    RCLCPP_INFO(node->get_logger(), "Number of Spheres in IRM: %lu", sphere_color.size());

    RCLCPP_INFO(node->get_logger(), 
      "All the poses have been processed. Now saving data to an inverse Reachability Map.");

    hdf5_dataset::Hdf5Dataset irm_h5(filename);
    irm_h5.saveReachMapsToDataset(base_trns_col, sphere_color, res);

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