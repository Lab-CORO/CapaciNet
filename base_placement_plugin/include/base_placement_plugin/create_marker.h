#ifndef CREATE_MARKER_H
#define CREATE_MARKER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.hpp>

#include <map>
#include <vector>
#include <string>
#include <memory>

typedef std::multimap<std::vector<double>, geometry_msgs::msg::Pose> BasePoseJoint;

class CreateMarker {
public:
  CreateMarker(std::shared_ptr<rclcpp::Node> node, const std::string& group_name);

  bool makeArmMarker(BasePoseJoint baseJoints, 
                     std::vector<visualization_msgs::msg::InteractiveMarker>& iMarkers, 
                     bool show_unreachable_models);
  
  bool makeRobotMarker(BasePoseJoint baseJoints, 
                       std::vector<visualization_msgs::msg::InteractiveMarker>& iMarkers, 
                       bool show_unreachable_models);
  
  bool checkEndEffector();
  
  visualization_msgs::msg::MarkerArray getDefaultMarkers();

  /**
   * @brief Forward Kinematics computation (to be implemented by user)
   *
   * TODO: Implement FK using your robot's kinematic library (e.g., KDL, MoveIt)
   * This method should compute the poses of all robot links given joint positions.
   *
   * @param joint_positions Joint angles/positions for the robot
   * @param link_poses Output vector of poses for each link
   * @return true if FK computation succeeded, false otherwise
   *
   * Current status: Stub implementation returns mock data
   */
  bool get_fk(const std::vector<double>& joint_positions,
              std::vector<geometry_msgs::msg::Pose>& link_poses);

  /**
   * @brief Inverse Kinematics computation
   *
   * @param target_pose Desired end-effector pose
   * @param joint_solution Output joint angles that achieve the target pose
   * @return true if IK solution found, false otherwise
   */
  bool get_ik(const geometry_msgs::msg::Pose& target_pose,
              std::vector<double>& joint_solution);

private:
  void discardUnreachableModels(BasePoseJoint& baseJoints);
  
  void makeIntMarkers(BasePoseJoint& basePJoints, 
                      bool arm_only, 
                      std::vector<visualization_msgs::msg::InteractiveMarker>& iMarkers);
  
  void updateJointState(const std::vector<double>& joint_soln, 
                        sensor_msgs::msg::JointState& joint_state);
  
  void getFullLinkNames(std::vector<std::string>& full_link_names, bool arm_only);
  void getArmLinks(std::vector<std::string>& arm_links);
  void getEELinks(std::vector<std::string>& ee_links);
  
  void makeIntMarkerControl(const geometry_msgs::msg::Pose& base_pose, 
                           const std::vector<double>& joint_soln,
                           bool arm_only, 
                           bool is_reachable, 
                           visualization_msgs::msg::InteractiveMarkerControl& robotModelControl);
  
  void createInteractiveMarker(const geometry_msgs::msg::Pose& base_pose, 
                              const std::vector<double>& joint_soln,
                              const int& num, 
                              bool arm_only, 
                              bool is_reachable,
                              visualization_msgs::msg::InteractiveMarker& iMarker);
  
  void updateMarkers(const geometry_msgs::msg::Pose& base_pose, 
                    bool is_reachable, 
                    Eigen::Affine3d tf_first_link_to_root, 
                    visualization_msgs::msg::MarkerArray& markers);

  /**
   * @brief Mock marker generation (to be replaced with URDF mesh loading)
   *
   * TODO: Replace with actual robot mesh loading from URDF
   * Currently generates simple geometric shapes (cylinders, spheres) as placeholders
   *
   * @param joint_positions Joint positions for visualization
   * @param markers Output marker array for RViz
   */
  void generateMockMarkers(const std::vector<double>& joint_positions,
                          visualization_msgs::msg::MarkerArray& markers);

  std::shared_ptr<rclcpp::Node> node_;
  std::string group_name_;
  std::string parent_link_;
  std::string end_effector_name_;

  /**
   * Robot configuration - Should be loaded from URDF/robot_description parameter
   *
   * TODO: Load from robot_description ROS parameter instead of hardcoded values
   * Current status: Hardcoded mock values in constructor
   */
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  std::vector<std::string> arm_link_names_;
  std::vector<std::string> ee_link_names_;
};

#endif // CREATE_MARKER_H