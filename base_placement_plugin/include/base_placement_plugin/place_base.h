#ifndef PLACE_BASE_H_
#define PLACE_BASE_H_

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>

#include <base_placement_plugin/create_marker.h>
#include <base_placement_plugin/add_robot_base.h>

#include "curobo_msgs/srv/ik.hpp"

// ROS2 action and service interfaces
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <base_placement_interfaces/action/find_base.hpp>
#include <base_placement_interfaces/srv/update_reachability_map.hpp>
#include <base_placement_interfaces/srv/get_union_map.hpp>
#include <base_placement_interfaces/srv/update_parameters.hpp>
#include <base_placement_interfaces/srv/add_named_pose.hpp>
#include <base_placement_interfaces/srv/remove_named_pose.hpp>
#include <base_placement_interfaces/srv/clear_maps.hpp>
#include <base_placement_interfaces/srv/get_base_poses.hpp>

#include <QObject>
#include <QTimer>
#include <QtConcurrent>
#include <QFuture>

#include <memory>
#include <string>
#include <vector>
#include <map>

/*!
 *  \brief     Class for setting up the Base Placement Environment.
 *  \details   The PlaceBase Class handles all the interactions between the widget and the actual task.
 *             This Class inherits from the QObject superclass.
 *             The concept of this class is to initialize all the necessary parameters for generating 
 *             Optimal Base Locations from task poses.
 *  \author    Abhijit Makhal (adapted to ROS2)
 */
class PlaceBase : public QObject
{
  Q_OBJECT

public:
  //! Constructor for the Base Placement Planner.
  PlaceBase(std::shared_ptr<rclcpp::Node> node, QObject* parent = 0);
  //! Virtual Destructor for the Base Placement planner.
  virtual ~PlaceBase();
  //! Initialization of the necessary parameters.
  void init();
  typedef struct {
    std_msgs::msg::Header header;
    geometry_msgs::msg::Point32 point;
    float ri;
    std::vector<geometry_msgs::msg::Pose> poses;
  }WsSphere;

  typedef struct{
    std_msgs::msg::Header header;
    float resolution;
    std::vector<PlaceBase::WsSphere> ws_spheres;
  }WorkSpace;

public Q_SLOTS:
  //! Getting the reachability data from widget
  void setReachabilityData(std::multimap<std::vector<double>, std::vector<double>> PoseCollection,
                           std::multimap<std::vector<double>, double> SphereCollection, 
                           float resolution);

  //! Showing the InverseReachability Map
  void ShowUnionMap(bool show_map);

  //! Clearing the InverseReachability Map
  void clearUnionMap(bool clear_map);

  //! Showing the base locations by arrow
  void showBaseLocationsbyArrow(std::vector<geometry_msgs::msg::Pose> po);

  //! Showing the base locations by robot model
  void showBaseLocationsbyRobotModel(std::vector<geometry_msgs::msg::Pose> po);

  //! Showing the base locations by only arm
  void showBaseLocationsbyArmModel(std::vector<geometry_msgs::msg::Pose> po);

  //! Get the Way-Points from the RViz environment and use them to find base.
  bool findbase(std::vector<geometry_msgs::msg::Pose> grasp_poses);

  //! Sending the initial marker location and list of base Placement Method and visualization methods
  void initRvizDone();

  //! Function for setting base placement main function to a separate thread.
  void BasePlacementHandler(std::vector<geometry_msgs::msg::Pose> waypoints);

  //! Setting the Base Placement Parameters
  void setBasePlaceParams(int base_loc_size, int high_score_sp_);

  //! Getting user Defined base placement method
  void getSelectedMethod(int index);

  //! Getting user Defined visualization method
  void getSelectedOpType(int op_index);

  //! Getting user Defined robot model
  void getSelectedRobotGroup(int model_index);

  //! Getting user Defined unreachable model shown method
  void getShowUreachModels(bool show_u_models);

  //! Getting the user Defined base poses
  void getBasePoses(std::vector<geometry_msgs::msg::Pose> base_poses);

Q_SIGNALS:
  //! Signal for initial marker frame
  void getinitialmarkerFrame_signal(const tf2::Transform trns);

  //! Let the RQT Widget know that a Base Placement Process has started.
  void basePlacementProcessStarted();

  //! Let the RQT Widget know that Base Placement Process has finished.
  void basePlacementProcessFinished();

  //! Let the RQT Widget know that Base Placement Process completed so it can show finish message
  void basePlacementProcessCompleted(double score);

  //! Send the Method groups to the GUI
  void sendBasePlaceMethods_signal(std::vector<std::string> method_names);

  //! Send the output types to the GUI
  void sendOuputType_signal(std::vector<std::string> output_type);

  //! Send the output types to the GUI
  void sendGroupType_signal(std::vector<std::string> group_names);

  //! Send the selected group
  void sendSelectedGroup_signal(std::string group_name);

protected:
  //! Base Placement by PCA
  void findBaseByPCA();

  //! Base Placement by GraspReachabilityScore
  void findBaseByGraspReachabilityScore();

  //! Base Placement by IKSolutionScore
  void findBaseByIKSolutionScore();

  //! Base Placement by VerticalRobotModel
  void findBaseByVerticalRobotModel();

  //! Base Placement by VerticalRobotModel
  void findBaseByUserIntuition();

  //! Selecting the Base Placement Method
  void BasePlaceMethodHandler();

  //! Selecting the Visualization Method
  void OuputputVizHandler(std::vector<geometry_msgs::msg::Pose> po);

  // Transforming reachability data towards robot base
  void transformToRobotbase(std::multimap<std::vector<double>, std::vector<double>> armBasePoses,
                            std::multimap<std::vector<double>, std::vector<double>>& robotBasePoses);

  void transformFromRobotbaseToArmBase(const geometry_msgs::msg::Pose& base_pose, 
                                       geometry_msgs::msg::Pose& arm_base_pose);
  
  void createSpheres(std::multimap<std::vector<double>, std::vector<double>> basePoses,
                     std::map<std::vector<double>, double>& spColor, 
                     std::vector<std::vector<double>>& highScoredSp, 
                     bool reduce_D);

  double calculateScoreForRobotBase(std::vector<geometry_msgs::msg::Pose>& grasp_poses, 
                                    std::vector<geometry_msgs::msg::Pose>& base_poses);
  
  double calculateScoreForArmBase(std::vector<geometry_msgs::msg::Pose>& grasp_poses, 
                                  std::vector<geometry_msgs::msg::Pose>& base_poses);

  bool loadRobotModel();
  bool checkforRobotModel();

  void getRobotGroups(std::vector<std::string>& groups);

  bool isIkSuccesswithTransformedBase(const geometry_msgs::msg::Pose& base_pose, const geometry_msgs::msg::Pose& grasp_pose, std::vector<double>& joint_soln,
                                      int& numOfSolns);

  // ROS2 node
  std::shared_ptr<rclcpp::Node> node_;

  int selected_method_;
  int selected_op_type_;
  std::string selected_group_;

  //! Number of base place locations to show
  int BASE_LOC_SIZE_;
  //! Number of High Scoring Spheres
  int HIGH_SCORE_SP_;

  //! Vector for method_names
  std::vector<std::string> method_names_;
  //! Vector for output visualization
  std::vector<std::string> output_type_;
  //! Vector for robot groups
  std::vector<std::string> group_names_;
  
  //! Taking the reachability data from the file
  std::multimap<std::vector<double>, std::vector<double>> PoseColFilter;
  std::multimap<std::vector<double>, double> SphereCol;
  float res;

  //! Reachability data for the union map
  std::multimap<std::vector<double>, std::vector<double>> baseTrnsCol;
  std::map<std::vector<double>, double> sphereColor;

  //! sphere centers based on their scores
  std::vector<std::vector<double>> highScoreSp;
  //! Vector for storing final base poses
  std::vector<geometry_msgs::msg::Pose> final_base_poses;
  std::vector<geometry_msgs::msg::Pose> final_base_poses_user;

  //! Vector for grasp poses
  std::vector<geometry_msgs::msg::Pose> GRASP_POSES_;

  //! Reachability data for the union map
  std::multimap<std::vector<double>, std::vector<double>> robot_PoseColfilter;

  double score_;
  geometry_msgs::msg::Pose best_pose_;

  // Show unreachable models
  bool show_ureach_models_;

  // Pointer for robot marker
  std::shared_ptr<CreateMarker> mark_;
  
  // TODO: Replace with proper robot model representation
  // For now, using joint names and link names
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;

  rclcpp::Client<curobo_msgs::srv::Ik>::SharedPtr client_ik;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  // ============================================================
  // ACTION AND SERVICE CLIENTS FOR NEW ARCHITECTURE
  // ============================================================

  //! Type aliases for action client
  using FindBaseAction = base_placement_interfaces::action::FindBase;
  using GoalHandleFindBase = rclcpp_action::ClientGoalHandle<FindBaseAction>;

  //! Action client for find_base
  rclcpp_action::Client<FindBaseAction>::SharedPtr find_base_action_client_;

  //! Service clients
  rclcpp::Client<base_placement_interfaces::srv::UpdateReachabilityMap>::SharedPtr update_reachability_client_;
  rclcpp::Client<base_placement_interfaces::srv::GetUnionMap>::SharedPtr get_union_map_client_;
  rclcpp::Client<base_placement_interfaces::srv::UpdateParameters>::SharedPtr update_parameters_client_;
  rclcpp::Client<base_placement_interfaces::srv::AddNamedPose>::SharedPtr add_named_pose_client_;
  rclcpp::Client<base_placement_interfaces::srv::RemoveNamedPose>::SharedPtr remove_named_pose_client_;
  rclcpp::Client<base_placement_interfaces::srv::ClearMaps>::SharedPtr clear_maps_client_;
  rclcpp::Client<base_placement_interfaces::srv::GetBasePoses>::SharedPtr get_base_poses_client_;

  //! Callback group for service clients
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;

  //! Goal handle for tracking action execution
  std::shared_ptr<GoalHandleFindBase> goal_handle_;

  //! Action callbacks
  void goalResponseCallback(const GoalHandleFindBase::SharedPtr& goal_handle);
  void feedbackCallback(
    GoalHandleFindBase::SharedPtr,
    const std::shared_ptr<const FindBaseAction::Feedback> feedback);
  void resultCallback(const GoalHandleFindBase::WrappedResult& result);

};

#endif  // PLACE_BASE_H_