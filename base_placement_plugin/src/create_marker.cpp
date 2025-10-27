#include "base_placement_plugin/create_marker.h"
#include <ctime>
#include <algorithm>

double unifRand() {
  return rand() / double(RAND_MAX);
}

CreateMarker::CreateMarker(std::shared_ptr<rclcpp::Node> node, const std::string& group_name)
  : node_(node), group_name_(group_name) {

  // TODO: Load configuration from URDF or robot_description ROS2 parameter
  // Current implementation uses hardcoded mock values
  // To implement: Parse robot_description parameter and extract joint/link names
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  link_names_ = {"base_link", "link1", "link2", "link3", "link4", "link5", "link6"};
  arm_link_names_ = {"link1", "link2", "link3", "link4", "link5", "link6"};
  ee_link_names_ = {"ee_link"};
  parent_link_ = "base_link";
  end_effector_name_ = "gripper";

  RCLCPP_INFO(node_->get_logger(), "CreateMarker initialized for group: %s", group_name_.c_str());
}

bool CreateMarker::checkEndEffector() {
  return !end_effector_name_.empty();
}

void CreateMarker::discardUnreachableModels(BasePoseJoint& baseJoints) {
  for (BasePoseJoint::iterator it = baseJoints.begin(); it != baseJoints.end();) {
    std::vector<double> joint_soln = it->first;
    if (std::equal(joint_soln.begin() + 1, joint_soln.end(), joint_soln.begin())) {
      BasePoseJoint::iterator save = it;
      ++save;
      baseJoints.erase(it);
      it = save;
    } else {
      ++it;
    }
  }
}

bool checkForJointSoln(const std::vector<double>& soln) {
  if (soln.empty()) return false;
  return std::equal(soln.begin() + 1, soln.end(), soln.begin());
}

void CreateMarker::updateJointState(const std::vector<double>& joint_soln, 
                                    sensor_msgs::msg::JointState& joint_state) {
  joint_state.header.stamp = node_->now();
  joint_state.header.frame_id = "base_link";
  joint_state.name = joint_names_;
  joint_state.position = joint_soln;
}

void CreateMarker::getArmLinks(std::vector<std::string>& arm_links) {
  arm_links = arm_link_names_;
}

void CreateMarker::getEELinks(std::vector<std::string>& ee_links) {
  ee_links = ee_link_names_;
}

void CreateMarker::getFullLinkNames(std::vector<std::string>& full_link_names, bool arm_only) {
  if (arm_only) {
    full_link_names = arm_link_names_;
    full_link_names.insert(full_link_names.end(), ee_link_names_.begin(), ee_link_names_.end());
    parent_link_ = "base_link"; // Parent du premier lien du bras
  } else {
    full_link_names = link_names_;
    parent_link_ = link_names_[0];
  }
}

void CreateMarker::generateMockMarkers(const std::vector<double>& /* joint_positions */,
                                       visualization_msgs::msg::MarkerArray& markers) {
  // TODO: Replace with actual mesh loading from URDF
  // Current implementation generates simple geometric shapes (cylinders, spheres)
  // as placeholders for robot links
  // To implement: Load mesh files from URDF and use MESH_RESOURCE markers

  markers.markers.clear();
  
  for (size_t i = 0; i < link_names_.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = node_->now();
    marker.ns = "robot_links";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Position mock basée sur l'index du lien
    marker.pose.position.x = i * 0.1;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Taille
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.1;
    
    // Couleur par défaut
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0f;
    
    marker.lifetime = rclcpp::Duration::from_seconds(40.0);
    
    markers.markers.push_back(marker);
  }
}

void CreateMarker::updateMarkers(const geometry_msgs::msg::Pose& base_pose, 
                                 bool is_reachable, 
                                 Eigen::Affine3d tf_first_link_to_root, 
                                 visualization_msgs::msg::MarkerArray& markers) {
  geometry_msgs::msg::TransformStamped pose;
  pose.transform.translation.x = base_pose.position.x;
  pose.transform.translation.y = base_pose.position.y;
  pose.transform.translation.z = base_pose.position.z;
  pose.transform.rotation = base_pose.orientation;

  Eigen::Affine3d base_tf = tf2::transformToEigen(pose);
  
  double r = unifRand();
  double g = unifRand();
  double b = unifRand();
  
  for (std::size_t j = 0; j < markers.markers.size(); j++) {
    markers.markers[j].header.frame_id = "base_link";
    markers.markers[j].header.stamp = node_->now();
    markers.markers[j].ns = "robot_links";
    markers.markers[j].lifetime = rclcpp::Duration::from_seconds(40.0);
    
    if (!is_reachable) {
      markers.markers[j].color.r = 1.0f;
      markers.markers[j].color.g = 0.0f;
      markers.markers[j].color.b = 0.0f;
      markers.markers[j].color.a = 1.0f;
    } else {
      markers.markers[j].color.r = static_cast<float>(r);
      markers.markers[j].color.g = static_cast<float>(g);
      markers.markers[j].color.b = static_cast<float>(b);
      markers.markers[j].color.a = 0.7f;
    }
    pose.transform.translation.x = markers.markers[j].pose.position.x;
    pose.transform.translation.y = markers.markers[j].pose.position.y;
    pose.transform.translation.z = markers.markers[j].pose.position.z;
    pose.transform.rotation = markers.markers[j].pose.orientation;
    Eigen::Affine3d link_marker = tf2::transformToEigen(pose);
    Eigen::Affine3d tf_link_in_root = tf_first_link_to_root * link_marker;
    geometry_msgs::msg::Pose new_marker_pose = tf2::toMsg(base_tf * tf_link_in_root);
    markers.markers[j].pose = new_marker_pose;
  }
}

void CreateMarker::makeIntMarkerControl(const geometry_msgs::msg::Pose& base_pose, 
                                        const std::vector<double>& joint_soln,
                                        bool arm_only, 
                                        bool is_reachable, 
                                        visualization_msgs::msg::InteractiveMarkerControl& robotModelControl) {
  
  sensor_msgs::msg::JointState joint_state;
  updateJointState(joint_soln, joint_state);
  
  std::vector<std::string> full_link_names;
  getFullLinkNames(full_link_names, arm_only);
  
  visualization_msgs::msg::MarkerArray full_link_markers;
  generateMockMarkers(joint_soln, full_link_markers);

  // TODO: Compute real transformation using Forward Kinematics
  // Current implementation uses identity transform (no actual FK computation)
  // To implement: Call get_fk() to compute real link poses from joint_soln
  Eigen::Affine3d tf_root_to_first_link = Eigen::Affine3d::Identity();
  Eigen::Affine3d tf_first_link_to_root = tf_root_to_first_link.inverse();
  
  updateMarkers(base_pose, is_reachable, tf_first_link_to_root, full_link_markers);
  
  for (int i = 0; i < static_cast<int>(full_link_markers.markers.size()); ++i) {
    robotModelControl.markers.push_back(full_link_markers.markers[i]);
  }
  
  robotModelControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
  robotModelControl.always_visible = true;
}

void CreateMarker::createInteractiveMarker(const geometry_msgs::msg::Pose& base_pose, 
                                           const std::vector<double>& joint_soln,
                                           const int& num, 
                                           bool arm_only, 
                                           bool is_reachable,
                                           visualization_msgs::msg::InteractiveMarker& iMarker) {
  iMarker.header.frame_id = "base_link";
  iMarker.pose = base_pose;
  iMarker.scale = 0.2;
  
  std::string name = "robot_model";
  std::string description = "robot_desc";
  iMarker.name = name + std::to_string(num);
  iMarker.description = description + std::to_string(num);
  
  visualization_msgs::msg::InteractiveMarkerControl robotModelControl;
  makeIntMarkerControl(base_pose, joint_soln, arm_only, is_reachable, robotModelControl);
  iMarker.controls.push_back(robotModelControl);
}

void CreateMarker::makeIntMarkers(BasePoseJoint& basePJoints, 
                                  bool arm_only, 
                                  std::vector<visualization_msgs::msg::InteractiveMarker>& iMarkers) {
  iMarkers.clear();
  
  int i = 0;
  for (BasePoseJoint::iterator it = basePJoints.begin(); it != basePJoints.end(); ++it) {
    bool is_reachable = true;
    geometry_msgs::msg::Pose base_pose = it->second;
    std::vector<double> joint_soln = it->first;
    
    if (checkForJointSoln(joint_soln)) {
      is_reachable = false;
    }
    
    visualization_msgs::msg::InteractiveMarker iMarker;
    createInteractiveMarker(base_pose, joint_soln, i, arm_only, is_reachable, iMarker);
    iMarkers.push_back(iMarker);
    i++;
  }
}

bool CreateMarker::makeRobotMarker(BasePoseJoint baseJoints, 
                                   std::vector<visualization_msgs::msg::InteractiveMarker>& iMarkers, 
                                   bool show_unreachable_models) {
  if (!show_unreachable_models) {
    discardUnreachableModels(baseJoints);
  }
  makeIntMarkers(baseJoints, false, iMarkers);
  return true;
}

bool CreateMarker::makeArmMarker(BasePoseJoint baseJoints, 
                                 std::vector<visualization_msgs::msg::InteractiveMarker>& iMarkers, 
                                 bool show_unreachable_models) {
  if (!show_unreachable_models) {
    discardUnreachableModels(baseJoints);
  }
  makeIntMarkers(baseJoints, true, iMarkers);
  return true;
}

visualization_msgs::msg::MarkerArray CreateMarker::getDefaultMarkers() {
  std::vector<double> default_joint_positions(joint_names_.size(), 0.0);
  visualization_msgs::msg::MarkerArray full_link_markers;
  generateMockMarkers(default_joint_positions, full_link_markers);
  return full_link_markers;
}

// TODO: User implementation required - Forward Kinematics stub
bool CreateMarker::get_fk(const std::vector<double>& /* joint_positions */,
                          std::vector<geometry_msgs::msg::Pose>& link_poses) {
  // Stub implementation - Replace with your robot's FK library (KDL, MoveIt, etc.)
  // This method should compute link poses from joint angles using robot kinematics
  RCLCPP_WARN(node_->get_logger(), "get_fk() not implemented - using identity transforms");
  
  link_poses.clear();
  for (size_t i = 0; i < link_names_.size(); ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = i * 0.1;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    link_poses.push_back(pose);
  }
  
  return true;
}

bool CreateMarker::get_ik(const geometry_msgs::msg::Pose& /* target_pose */,
                          std::vector<double>& joint_solution) {
  // Placeholder - implémenter avec votre bibliothèque IK/FK
  RCLCPP_WARN(node_->get_logger(), "get_ik() not implemented - returning zero configuration");
  
  joint_solution.clear();
  joint_solution.resize(joint_names_.size(), 0.0);
  
  return false; // Retourner true quand implémenté
}