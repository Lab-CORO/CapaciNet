#include <base_placement_plugin/place_base.h>

#include <math.h>
#include <chrono>
#include <Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.hpp>

#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <visualization_msgs/msg/marker_array.hpp>

// #include <map_creator/WorkSpace.h>
#include "../include/base_placement_plugin/sphere_discretization.h"
// #include <map_creator/kinematics.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <visualization_msgs/msg/interactive_marker.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

using namespace std::chrono_literals;


PlaceBase::PlaceBase(std::shared_ptr<rclcpp::Node> node, QObject* parent)
  : QObject(parent), node_(node)
{
  init();

  client_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->client_ik = node_->create_client<curobo_msgs::srv::Ik>("/curobo_ik/ik_pose", rmw_qos_profile_services_default,
                                                                  client_cb_group_);

  // Create callback group for service clients
  service_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Initialize action client
  find_base_action_client_ = rclcpp_action::create_client<FindBaseAction>(
    node_, "find_base", service_cb_group_);

  // Initialize service clients
  update_reachability_client_ = node_->create_client<base_placement_interfaces::srv::UpdateReachabilityMap>(
    "update_reachability_map", rmw_qos_profile_services_default, service_cb_group_);

  get_union_map_client_ = node_->create_client<base_placement_interfaces::srv::GetUnionMap>(
    "get_union_map", rmw_qos_profile_services_default, service_cb_group_);

  update_parameters_client_ = node_->create_client<base_placement_interfaces::srv::UpdateParameters>(
    "update_parameters", rmw_qos_profile_services_default, service_cb_group_);

  add_named_pose_client_ = node_->create_client<base_placement_interfaces::srv::AddNamedPose>(
    "add_named_pose", rmw_qos_profile_services_default, service_cb_group_);

  remove_named_pose_client_ = node_->create_client<base_placement_interfaces::srv::RemoveNamedPose>(
    "remove_named_pose", rmw_qos_profile_services_default, service_cb_group_);

  clear_maps_client_ = node_->create_client<base_placement_interfaces::srv::ClearMaps>(
    "clear_maps", rmw_qos_profile_services_default, service_cb_group_);

  get_base_poses_client_ = node_->create_client<base_placement_interfaces::srv::GetBasePoses>(
    "get_base_poses", rmw_qos_profile_services_default, service_cb_group_);

  RCLCPP_INFO(node_->get_logger(), "PlaceBase initialized with action/service clients");

}

PlaceBase::~PlaceBase()
{
}

void PlaceBase::init()
{
  show_ureach_models_ = false;
  selected_method_ = 0;
  selected_op_type_ = 0;
  BASE_LOC_SIZE_ = 5;
  HIGH_SCORE_SP_ = 10;
  score_ = 0.0;
  res = 0.0f;
}

void PlaceBase::BasePlacementHandler(std::vector<geometry_msgs::msg::Pose> waypoints)
{
  RCLCPP_INFO(node_->get_logger(), "Starting concurrent process for Base Placement");
  QFuture<void> future = QtConcurrent::run(this, &PlaceBase::findbase, waypoints);
}

void PlaceBase::initRvizDone()
{
  RCLCPP_INFO(node_->get_logger(), "RViz is done now we need to emit the signal");

  tf2::Vector3 vec(0, 0, 0);
  tf2::Quaternion quat(0, 0, 0, 1);
  quat.normalize();
  tf2::Transform trns;
  trns.setOrigin(vec);
  trns.setRotation(quat);

  Q_EMIT getinitialmarkerFrame_signal(trns);

  RCLCPP_INFO(node_->get_logger(), "Sending the method list");
  method_names_.push_back("PrincipalComponentAnalysis");
  method_names_.push_back("GraspReachabilityScore");
  method_names_.push_back("IKSolutionScore");
  method_names_.push_back("VerticalRobotModel");
  method_names_.push_back("UserIntuition");
  Q_EMIT sendBasePlaceMethods_signal(method_names_);

  RCLCPP_INFO(node_->get_logger(), "Sending the output visualization method list");
  output_type_.push_back("Arrows");
  output_type_.push_back("Manipulator");
  output_type_.push_back("RobotModel");
  Q_EMIT sendOuputType_signal(output_type_);
}

void PlaceBase::getShowUreachModels(bool show_u_models)
{
  show_ureach_models_ = show_u_models;
  RCLCPP_INFO(node_->get_logger(), "Show unreachable model changing");
}

void PlaceBase::getSelectedRobotGroup(int group_index)
{
  if (group_index < 0 || group_index >= static_cast<int>(group_names_.size()))
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid group index: %d", group_index);
    return;
  }
  
  selected_group_ = group_names_[group_index];
  RCLCPP_INFO(node_->get_logger(), "Selected Group: %s", group_names_[group_index].c_str());
  
  mark_ = std::make_shared<CreateMarker>(node_, selected_group_);
  if (!mark_->checkEndEffector())
  {
    RCLCPP_ERROR(node_->get_logger(), "Is your selected group a manipulator??");
    mark_.reset();
  }
  Q_EMIT sendSelectedGroup_signal(selected_group_);
}

void PlaceBase::getSelectedMethod(int index)
{
  selected_method_ = index;
  RCLCPP_INFO(node_->get_logger(), "selected_method: %s", method_names_[index].c_str());
  
  if (index == 3 || index == 4)
  {
    if (!checkforRobotModel())
    {
      RCLCPP_ERROR(node_->get_logger(), 
        "The base placement method you selected needs robot model to be loaded.");
    }
  }
}

void PlaceBase::getSelectedOpType(int op_index)
{
  selected_op_type_ = op_index;
  RCLCPP_INFO(node_->get_logger(), "selected visualization method: %s", 
    output_type_[op_index].c_str());
  
  if (selected_op_type_ == 1 || selected_op_type_ == 2)
  {
    if (!checkforRobotModel())
    {
      RCLCPP_ERROR(node_->get_logger(), 
        "The visualization method you selected needs robot model to be loaded.");
    }
  }
}

bool PlaceBase::checkforRobotModel()
{
  if (joint_names_.empty())
  {
    if (loadRobotModel())
    {
      group_names_.clear();
      group_names_.push_back("manipulator");
      group_names_.push_back("arm");
      
      Q_EMIT sendGroupType_signal(group_names_);
      RCLCPP_INFO(node_->get_logger(), "Please select your manipulator group.");
      return true;
    }
  }
  else
  {
    return true;
  }
  return false;
}

bool PlaceBase::loadRobotModel()
{
  // TODO: Implement robot model loading from robot_description parameter
  RCLCPP_WARN(node_->get_logger(), 
    "loadRobotModel() is a stub - implement robot description parsing");
  
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  link_names_ = {"base_link", "link1", "link2", "link3", "link4", "link5", "link6"};
  
  return true;
}

void PlaceBase::getBasePoses(std::vector<geometry_msgs::msg::Pose> base_poses)
{
  final_base_poses_user = base_poses;
}

void PlaceBase::setBasePlaceParams(int base_loc_size, int high_score_sp)
{
  RCLCPP_INFO(node_->get_logger(),
    "Base Placement parameters from UI:\n Number of base locations: %d\n Number of HighScore Spheres: %d",
    base_loc_size, high_score_sp);

  // Validate parameters locally
  if (base_loc_size <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Please provide a valid number of how many base locations do you need.");
    return;
  }

  if (high_score_sp <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Please provide a valid number of how many spheres do you need to create valid base poses");
    return;
  }

  // Update local variables
  BASE_LOC_SIZE_ = base_loc_size;
  HIGH_SCORE_SP_ = high_score_sp;

  // Update parameters on server via service
  if (!update_parameters_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "update_parameters service not available, parameters only updated locally");
    return;
  }

  auto request = std::make_shared<base_placement_interfaces::srv::UpdateParameters::Request>();
  request->method_index = selected_method_;
  request->num_base_locations = base_loc_size;
  request->num_high_score_spheres = high_score_sp;

  auto result_future = update_parameters_client_->async_send_request(request);

  // Don't block - parameters are already set locally
  // The service call ensures the server is also updated
  RCLCPP_INFO(node_->get_logger(), "Sent parameter update request to server");
}

void PlaceBase::createSpheres(std::multimap<std::vector<double>, std::vector<double>> basePoses,
                              std::map<std::vector<double>, double>& spColor, 
                              std::vector<std::vector<double>>& highScoredSp, 
                              bool reduce_D)
{
//   kinematics::Kinematics k;
  std::vector<int> poseCount;
  poseCount.reserve(basePoses.size());
  
  for (std::multimap<std::vector<double>, std::vector<double>>::iterator it = basePoses.begin(); 
       it != basePoses.end(); ++it)
  {
    int num = basePoses.count(it->first);
    poseCount.push_back(num);
  }
  
  std::vector<int>::const_iterator it;
  it = max_element(poseCount.begin(), poseCount.end());
  int max_number = *it;
  it = min_element(poseCount.begin(), poseCount.end());
  int min_number = *it;
  
  if (reduce_D)
  {
    for (std::multimap<std::vector<double>, std::vector<double>>::iterator it = basePoses.begin(); 
         it != basePoses.end(); ++it)
    {
      if ((it->first)[2] < 0.051 && (it->first)[2] > -0.051)
      {
        geometry_msgs::msg::Pose prob_base_pose;
        prob_base_pose.position.x = (it->first)[0];
        prob_base_pose.position.y = (it->first)[1];
        prob_base_pose.position.z = (it->first)[2];
        prob_base_pose.orientation.w = 1.0;
        
        geometry_msgs::msg::Pose base_pose_at_arm;
        transformFromRobotbaseToArmBase(prob_base_pose, base_pose_at_arm);
        
        int num_of_solns = 0;
        for (size_t j = 0; j < GRASP_POSES_.size(); j++)
        {
          std::vector<double> joint_soln;
          int nsolns = 0;
          isIkSuccesswithTransformedBase(base_pose_at_arm, GRASP_POSES_[j], joint_soln, nsolns);
          num_of_solns += nsolns;
        }

        float d = (float(num_of_solns) / float(GRASP_POSES_.size() * 8)) * 100;
        spColor.insert(std::pair<std::vector<double>, double>(it->first, double(d)));
      }
    }
  }
  else
  {
    for (std::multimap<std::vector<double>, std::vector<double>>::iterator it = basePoses.begin(); 
         it != basePoses.end(); ++it)
    {
      float d = ((float(basePoses.count(it->first)) - min_number) / (max_number - min_number)) * 100;
      if (d > 1)
      {
        spColor.insert(std::pair<std::vector<double>, double>(it->first, double(d)));
      }
    }
  }
  
  std::multiset<std::pair<double, std::vector<double>>> scoreWithSp;
  for (std::map<std::vector<double>, double>::iterator it = spColor.begin(); 
       it != spColor.end(); ++it)
  {
    scoreWithSp.insert(std::pair<double, std::vector<double>>(it->second, it->first));
  }
  
  for (std::multiset<std::pair<double, std::vector<double>>>::reverse_iterator it = scoreWithSp.rbegin();
       it != scoreWithSp.rend(); ++it)
  {
    highScoredSp.push_back(it->second);
  }
}

double PlaceBase::calculateScoreForRobotBase(std::vector<geometry_msgs::msg::Pose>& grasp_poses, 
                                             std::vector<geometry_msgs::msg::Pose>& base_poses)
{
//   kinematics::Kinematics k;
  float total_score = 0;
  float max_score = 0;
  geometry_msgs::msg::Pose best_pose;
  
  for (size_t i = 0; i < base_poses.size(); i++)
  {
    geometry_msgs::msg::Pose base_pose_at_arm;
    transformFromRobotbaseToArmBase(base_poses[i], base_pose_at_arm);
    
    int num_of_solns = 0;
    for (size_t j = 0; j < grasp_poses.size(); j++)
    {
      std::vector<double> joint_soln;
      int nsolns = 0;
      isIkSuccesswithTransformedBase(base_pose_at_arm, grasp_poses[j], joint_soln, nsolns);
      num_of_solns += nsolns;
    }
    
    float d = (float(num_of_solns) / float(grasp_poses.size() * 8)) * 100;
    if (d > max_score)
    {
      max_score = d;
      best_pose = base_poses[i];
    }
    total_score += d;
  }
  
  best_pose_ = best_pose;
  double score = double(total_score / float(base_poses.size()));
  return score;
}

double PlaceBase::calculateScoreForArmBase(std::vector<geometry_msgs::msg::Pose>& grasp_poses, 
                                           std::vector<geometry_msgs::msg::Pose>& base_poses)
{
//   kinematics::Kinematics k;
  float total_score = 0;
  float max_score = 0;
  geometry_msgs::msg::Pose best_pose;
  
  for (size_t i = 0; i < base_poses.size(); i++)
  {
    int num_of_solns = 0;
    for (size_t j = 0; j < grasp_poses.size(); j++)
    {
      std::vector<double> joint_soln;
      int nsolns = 0;
      isIkSuccesswithTransformedBase(base_poses[i], grasp_poses[j], joint_soln, nsolns);
      num_of_solns += nsolns;
    }
    
    float d = (float(num_of_solns) / float(grasp_poses.size() * 8)) * 100;
    if (d > max_score)
    {
      max_score = d;
      best_pose = base_poses[i];
    }
    total_score += d;
  }
  
  best_pose_ = best_pose;
  double score = double(total_score / float(base_poses.size()));
  return score;
}

void PlaceBase::transformFromRobotbaseToArmBase(const geometry_msgs::msg::Pose& base_pose, 
                                                geometry_msgs::msg::Pose& arm_base_pose)
{
  // TODO: Implement with proper robot model
  RCLCPP_WARN_ONCE(node_->get_logger(), 
    "transformFromRobotbaseToArmBase() using identity transform - implement with robot model");
  
  Eigen::Affine3d trans_to_arm_parent = Eigen::Affine3d::Identity();
  geometry_msgs::msg::Transform base_transform;
  base_transform.translation.x =base_pose.position.x;
  base_transform.translation.y =base_pose.position.y;
  base_transform.translation.z =base_pose.position.z;
  base_transform.rotation = base_pose.orientation;
  Eigen::Affine3d base_pose_tf = tf2::transformToEigen(base_transform);
  arm_base_pose = tf2::toMsg(base_pose_tf * trans_to_arm_parent);
}

void PlaceBase::transformToRobotbase(std::multimap<std::vector<double>, std::vector<double>> armBasePoses,
                                     std::multimap<std::vector<double>, std::vector<double>>& robotBasePoses)
{
  RCLCPP_WARN_ONCE(node_->get_logger(), 
    "transformToRobotbase() using identity transform - implement with robot model");
  
  Eigen::Affine3d arm_to_root = Eigen::Affine3d::Identity();
  
  sphere_discretization::SphereDiscretization sd;
  for (std::multimap<std::vector<double>, std::vector<double>>::iterator it = armBasePoses.begin(); 
       it != armBasePoses.end(); ++it)
  {
    geometry_msgs::msg::Pose arm_base_pose;
    sd.convertVectorToPose(it->second, arm_base_pose);
    geometry_msgs::msg::Transform arm_transform;
    arm_transform.translation.x =arm_base_pose.position.x;
    arm_transform.translation.y =arm_base_pose.position.y;
    arm_transform.translation.z =arm_base_pose.position.z;
    arm_transform.rotation = arm_base_pose.orientation;
    Eigen::Affine3d arm_base_tf = tf2::transformToEigen(arm_transform);
    geometry_msgs::msg::Pose robot_base_pose = tf2::toMsg(arm_to_root * arm_base_tf);
    
    static const int arr[] = {1, 1, 1};
    std::vector<double> base_vec(arr, arr + sizeof(arr) / sizeof(arr[0]));
    std::vector<double> base_pose;
    sd.convertPoseToVector(robot_base_pose, base_pose);
    robotBasePoses.insert(std::pair<std::vector<double>, std::vector<double>>(base_vec, base_pose));
  }
}

bool PlaceBase::findbase(std::vector<geometry_msgs::msg::Pose> grasp_poses)
{
  Q_EMIT basePlacementProcessStarted();
  score_ = 0;

  if (grasp_poses.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Please provide at least one grasp pose.");
    Q_EMIT basePlacementProcessFinished();
    return false;
  }

  // Wait for action server to be available
  if (!find_base_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    Q_EMIT basePlacementProcessFinished();
    return false;
  }

  // ============================================================
  // IMPORTANT: Calculate union map BEFORE sending goal
  // ============================================================
  if (PoseColFilter.size() > 0 && selected_method_ != 4) {
    // UserIntuition (method 4) doesn't need reachability data
    sphere_discretization::SphereDiscretization sd;
    baseTrnsCol.clear();
    sphereColor.clear();
    highScoreSp.clear();

    RCLCPP_INFO(node_->get_logger(),
      "Calculating union map from %zu IRM poses with %zu task poses...",
      PoseColFilter.size(), grasp_poses.size());

    if (selected_method_ == 3) {
      // VerticalRobotModel needs robot base transformation
      robot_PoseColfilter.clear();
      transformToRobotbase(PoseColFilter, robot_PoseColfilter);
      sd.associatePose(baseTrnsCol, grasp_poses, robot_PoseColfilter, res);
      createSpheres(baseTrnsCol, sphereColor, highScoreSp, true);
    } else {
      // PCA, GraspReachabilityScore, IKSolutionScore use arm base
      sd.associatePose(baseTrnsCol, grasp_poses, PoseColFilter, res);
      createSpheres(baseTrnsCol, sphereColor, highScoreSp, false);
    }

    RCLCPP_INFO(node_->get_logger(),
      "Union map created: %zu poses, %zu spheres, %zu high-score spheres",
      baseTrnsCol.size(), sphereColor.size(), highScoreSp.size());

    // TODO: Send this data to server via update_reachability_map service
    // For now, we rely on the local computation
  }

  RCLCPP_INFO(node_->get_logger(), "Sending goal to base_placement_server with %zu task poses",
              grasp_poses.size());

  // Create and send goal
  auto goal_msg = FindBaseAction::Goal();

  // Convert Pose vector to PoseNamed vector
  for (size_t i = 0; i < grasp_poses.size(); ++i) {
    base_placement_interfaces::msg::PoseNamed named_pose;
    named_pose.name = "task_pose_" + std::to_string(i);
    named_pose.pose = grasp_poses[i];
    goal_msg.task_poses.push_back(named_pose);
  }

  goal_msg.method_index = selected_method_;
  goal_msg.num_base_locations = BASE_LOC_SIZE_;
  goal_msg.num_high_score_spheres = HIGH_SCORE_SP_;

  // If we have computed high-score spheres locally, send them to the server
  if (!highScoreSp.empty() && selected_method_ != 4) {
    goal_msg.use_provided_spheres = true;
    // Flatten the high-score sphere positions into a 1D array
    for (const auto& sphere : highScoreSp) {
      if (sphere.size() >= 3) {
        goal_msg.high_score_sphere_data.push_back(sphere[0]); // x
        goal_msg.high_score_sphere_data.push_back(sphere[1]); // y
        goal_msg.high_score_sphere_data.push_back(sphere[2]); // z
      }
    }
    RCLCPP_INFO(node_->get_logger(),
      "Sending %zu pre-computed high-score spheres to server",
      highScoreSp.size());
  } else {
    goal_msg.use_provided_spheres = false;
  }

  // If we have computed union map locally, send it to the server
  if (!baseTrnsCol.empty() && selected_method_ != 4) {
    goal_msg.use_provided_union_map = true;

    // Flatten the multimap data
    sphere_discretization::SphereDiscretization sd;
    std::vector<double> current_key;
    int pose_count = 0;

    for (auto it = baseTrnsCol.begin(); it != baseTrnsCol.end(); ++it) {
      // Check if we moved to a new key
      if (current_key.empty() || current_key != it->first) {
        // Save the count for the previous key
        if (!current_key.empty()) {
          goal_msg.base_trns_col_counts.push_back(pose_count);
        }

        // Start new key
        current_key = it->first;
        pose_count = 0;

        // Add key (sphere position)
        for (const auto& val : it->first) {
          goal_msg.base_trns_col_keys.push_back(val);
        }
      }

      // Add value (pose)
      geometry_msgs::msg::Pose pose;
      sd.convertVectorToPose(it->second, pose);
      goal_msg.base_trns_col_values.push_back(pose.position.x);
      goal_msg.base_trns_col_values.push_back(pose.position.y);
      goal_msg.base_trns_col_values.push_back(pose.position.z);
      goal_msg.base_trns_col_values.push_back(pose.orientation.x);
      goal_msg.base_trns_col_values.push_back(pose.orientation.y);
      goal_msg.base_trns_col_values.push_back(pose.orientation.z);
      goal_msg.base_trns_col_values.push_back(pose.orientation.w);

      pose_count++;
    }

    // Don't forget the last count
    if (!current_key.empty()) {
      goal_msg.base_trns_col_counts.push_back(pose_count);
    }

    RCLCPP_INFO(node_->get_logger(),
      "Sending union map with %zu poses across %zu spheres to server",
      baseTrnsCol.size(), goal_msg.base_trns_col_counts.size());
  } else {
    goal_msg.use_provided_union_map = false;
  }

  // Set up send options with callbacks
  auto send_goal_options = rclcpp_action::Client<FindBaseAction>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&PlaceBase::goalResponseCallback, this, std::placeholders::_1);

  send_goal_options.feedback_callback =
    std::bind(&PlaceBase::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback =
    std::bind(&PlaceBase::resultCallback, this, std::placeholders::_1);

  // Send the goal asynchronously
  RCLCPP_INFO(node_->get_logger(),
    "Sending goal: method=%d, num_base_locations=%d, num_high_score_spheres=%d",
    selected_method_, BASE_LOC_SIZE_, HIGH_SCORE_SP_);

  find_base_action_client_->async_send_goal(goal_msg, send_goal_options);

  RCLCPP_INFO(node_->get_logger(), "Goal sent, waiting for server response...");

  return true;
}

void PlaceBase::BasePlaceMethodHandler()
{
  switch (selected_method_)
  {
    case 0:
      findBaseByPCA();
      break;
    case 1:
      findBaseByGraspReachabilityScore();
      break;
    case 2:
      findBaseByIKSolutionScore();
      break;
    case 3:
      findBaseByVerticalRobotModel();
      break;
    case 4:
      findBaseByUserIntuition();
      break;
  }
}

void PlaceBase::OuputputVizHandler(std::vector<geometry_msgs::msg::Pose> po)
{
  switch (selected_op_type_)
  {
    case 0:
      showBaseLocationsbyArrow(po);
      break;
    case 1:
      showBaseLocationsbyArmModel(po);
      break;
    case 2:
      showBaseLocationsbyRobotModel(po);
      break;
  }
}

void PlaceBase::findBaseByUserIntuition()
{
  RCLCPP_INFO(node_->get_logger(), "Finding optimal base pose by user intuition.");
  double s = calculateScoreForRobotBase(GRASP_POSES_, final_base_poses_user);
  score_ = s;
  final_base_poses = final_base_poses_user;
}

void PlaceBase::findBaseByVerticalRobotModel()
{
  RCLCPP_INFO(node_->get_logger(), "Finding optimal ROBOT base pose by Vertical robot model.");
  std::vector<geometry_msgs::msg::Pose> base_poses;
  std::vector<geometry_msgs::msg::Pose> base_poses_user;
  
  int num_of_desired_sp = HIGH_SCORE_SP_ + 50;
  int num_of_sp = highScoreSp.size();
  
  if (BASE_LOC_SIZE_ > num_of_sp)
  {
    RCLCPP_ERROR(node_->get_logger(), "Desired base locations are too high. Please reduce it");
  }
  
  for (int i = 0; i < num_of_desired_sp; i += 4)
  {
    geometry_msgs::msg::Pose prob_base_pose;
    prob_base_pose.position.x = highScoreSp[i][0];
    prob_base_pose.position.y = highScoreSp[i][1];
    prob_base_pose.position.z = highScoreSp[i][2];
    prob_base_pose.orientation.x = 0;
    prob_base_pose.orientation.y = 0;
    prob_base_pose.orientation.z = 0;
    prob_base_pose.orientation.w = 1;
    base_poses.push_back(prob_base_pose);
  }

  for (int i = 0; i < BASE_LOC_SIZE_; ++i)
  {
    base_poses_user.push_back(base_poses[i]);
  }

  double s = calculateScoreForRobotBase(GRASP_POSES_, base_poses_user);
  score_ = s;
  final_base_poses = base_poses_user;
}

void PlaceBase::findBaseByPCA()
{
  RCLCPP_INFO(node_->get_logger(), "Finding optimal base pose by PCA.");

  // Validation: check if highScoreSp has enough elements
  if (highScoreSp.size() < static_cast<size_t>(BASE_LOC_SIZE_))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Not enough high score spheres! Required: %d, Available: %lu",
      BASE_LOC_SIZE_, highScoreSp.size());
    return;
  }

  // Validation: check if baseTrnsCol is not empty
  if (baseTrnsCol.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "baseTrnsCol is empty! Cannot proceed with PCA.");
    return;
  }

  sphere_discretization::SphereDiscretization sd;
  std::vector<geometry_msgs::msg::Pose> pose_scores;
  PlaceBase::WorkSpace ws;

  for (int i = 0; i < BASE_LOC_SIZE_; ++i)
  {
    // Additional safety check
    if (highScoreSp[i].size() < 3)
    {
      RCLCPP_ERROR(node_->get_logger(),
        "highScoreSp[%d] doesn't have 3 elements! Size: %lu", i, highScoreSp[i].size());
      continue;
    }

    PlaceBase::WsSphere wss;
    wss.point.x = highScoreSp[i][0];
    wss.point.y = highScoreSp[i][1];
    wss.point.z = highScoreSp[i][2];

    std::vector<double> basePose;
    basePose.push_back(highScoreSp[i][0]);
    basePose.push_back(highScoreSp[i][1]);
    basePose.push_back(highScoreSp[i][2]);

    std::multimap<std::vector<double>, std::vector<double>>::iterator it;
    for (it = baseTrnsCol.lower_bound(basePose); it != baseTrnsCol.upper_bound(basePose); ++it)
    {
      geometry_msgs::msg::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      wss.poses.push_back(pp);
    }

    // Only add sphere if it has poses
    if (!wss.poses.empty())
    {
      ws.ws_spheres.push_back(wss);
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(),
        "No poses found for sphere %d at position [%f, %f, %f]",
        i, wss.point.x, wss.point.y, wss.point.z);
    }
  }

  // Check if we have any valid spheres
  if (ws.ws_spheres.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "No valid spheres with poses found! Cannot proceed.");
    return;
  }

  for (size_t i = 0; i < ws.ws_spheres.size(); ++i)
  {
    geometry_msgs::msg::Pose final_base_pose;
    sd.findOptimalPosebyPCA(ws.ws_spheres[i].poses, final_base_pose);

    final_base_pose.position.x = ws.ws_spheres[i].point.x;
    final_base_pose.position.y = ws.ws_spheres[i].point.y;
    final_base_pose.position.z = ws.ws_spheres[i].point.z;
    geometry_msgs::msg::Transform final_transform;
    final_transform.translation.x = final_base_pose.position.x;
    final_transform.translation.y = final_base_pose.position.y;
    final_transform.translation.z = final_base_pose.position.z;
    final_transform.rotation = final_base_pose.orientation;
    Eigen::Affine3d final_base_tf = tf2::transformToEigen(final_transform);
    Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d(1, 0, 0)));
    geometry_msgs::msg::Pose new_base_pose = tf2::toMsg(final_base_tf * rx);

    pose_scores.push_back(new_base_pose);
  }

  double s = calculateScoreForArmBase(GRASP_POSES_, pose_scores);
  score_ = s;
  final_base_poses = pose_scores;
}

void PlaceBase::findBaseByGraspReachabilityScore()
{
  RCLCPP_INFO(node_->get_logger(), "Finding optimal base pose by GraspReachabilityScore.");
  sphere_discretization::SphereDiscretization sd;
//   kinematics::Kinematics kin;
  std::vector<geometry_msgs::msg::Pose> pose_scores;
  int numofSp = BASE_LOC_SIZE_;

  for (int i = 0; i < numofSp; i++)
  {
    std::vector<geometry_msgs::msg::Pose> probBasePoses;
    std::multimap<std::vector<double>, std::vector<double>>::iterator it;
    
    for (it = baseTrnsCol.lower_bound(highScoreSp[i]); 
         it != baseTrnsCol.upper_bound(highScoreSp[i]); ++it)
    {
      geometry_msgs::msg::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      probBasePoses.push_back(pp);
    }
    
    std::map<int, geometry_msgs::msg::Pose> basePoseWithHits;
    for (size_t j = 0; j < probBasePoses.size(); j++)
    {
      int numofHits = 0;
      for (size_t j1 = 0; j1 < GRASP_POSES_.size(); j1++)
      {
        int nsolns = 0;
        std::vector<double> joint_solns;
        numofHits += isIkSuccesswithTransformedBase(probBasePoses[j], GRASP_POSES_[j1], 
                                                        joint_solns, nsolns);
      }
      basePoseWithHits.insert(std::make_pair(numofHits, probBasePoses[j]));
    }
    
    std::map<int, geometry_msgs::msg::Pose>::iterator itr;
    itr = basePoseWithHits.end();
    --itr;
    pose_scores.push_back(itr->second);
  }
  
  double s = calculateScoreForArmBase(GRASP_POSES_, pose_scores);
  score_ = s;
  final_base_poses = pose_scores;
}

void PlaceBase::findBaseByIKSolutionScore()
{
  RCLCPP_INFO(node_->get_logger(), "Finding optimal base pose by IKSolutionScore.");
  sphere_discretization::SphereDiscretization sd;
//   kinematics::Kinematics kin;
  std::vector<geometry_msgs::msg::Pose> pose_scores;
  int numofSp = BASE_LOC_SIZE_;

  int max_solns = GRASP_POSES_.size() * 8;
  int min_solns = 0;

  for (int i = 0; i < numofSp; i++)
  {
    std::vector<geometry_msgs::msg::Pose> probBasePoses;
    std::multimap<std::vector<double>, std::vector<double>>::iterator it;
    
    for (it = baseTrnsCol.lower_bound(highScoreSp[i]); 
         it != baseTrnsCol.upper_bound(highScoreSp[i]); ++it)
    {
      geometry_msgs::msg::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      probBasePoses.push_back(pp);
    }
    
    std::map<double, geometry_msgs::msg::Pose> basePoseWithHits;
    for (size_t j = 0; j < probBasePoses.size(); j++)
    {
      int numofHits = 0;
      int solns = 0;
      for (size_t j1 = 0; j1 < GRASP_POSES_.size(); j1++)
      {
        int nsolns = 0;
        std::vector<double> joint_solns;
        numofHits += isIkSuccesswithTransformedBase(probBasePoses[j], GRASP_POSES_[j1], 
                                                        joint_solns, nsolns);
        solns += nsolns;
      }
      double basePlaceScore = (double(solns) - double(min_solns)) / 
                              (double(max_solns) - double(min_solns));
      basePoseWithHits.insert(std::make_pair(basePlaceScore, probBasePoses[j]));
    }
    
    std::map<double, geometry_msgs::msg::Pose>::iterator itr;
    itr = basePoseWithHits.end();
    --itr;
    pose_scores.push_back(itr->second);
  }
  
  double s = calculateScoreForArmBase(GRASP_POSES_, pose_scores);
  score_ = s;
  final_base_poses = pose_scores;
}

void PlaceBase::showBaseLocationsbyArrow(std::vector<geometry_msgs::msg::Pose> po)
{
  RCLCPP_INFO(node_->get_logger(), 
    "Showing Base Locations by Arrow: Arrows are pointing in Z direction");
  
  std::vector<geometry_msgs::msg::Pose> pose_arr;
  for (size_t i = 0; i < po.size(); i++)
  {
    tf2::Transform trns;
    tf2::Quaternion quat(po[i].orientation.x, po[i].orientation.y, 
                         po[i].orientation.z, po[i].orientation.w);
    tf2::Vector3 vec(po[i].position.x, po[i].position.y, po[i].position.z);
    trns.setOrigin(vec);
    trns.setRotation(quat);

    tf2::Quaternion quat2;
    quat2.setRPY(0, -M_PI / 2, 0);
    tf2::Vector3 vec2(0, 0, 0);
    tf2::Transform multiplier;
    multiplier.setOrigin(vec2);
    multiplier.setRotation(quat2);

    trns = trns * multiplier;
    tf2::Vector3 new_pose_vec = trns.getOrigin();
    tf2::Quaternion new_pose_quat = trns.getRotation();
    new_pose_quat.normalize();

    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = new_pose_vec[0];
    new_pose.position.y = new_pose_vec[1];
    new_pose.position.z = new_pose_vec[2];
    new_pose.orientation.x = new_pose_quat[0];
    new_pose.orientation.y = new_pose_quat[1];
    new_pose.orientation.z = new_pose_quat[2];
    new_pose.orientation.w = new_pose_quat[3];
    pose_arr.push_back(new_pose);
  }

  auto marker_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "visualization_marker_array", 1);
  
  visualization_msgs::msg::MarkerArray markerArr;
  for (size_t i = 0; i < po.size(); ++i)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = node_->now();
    marker.ns = "points";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.scale.x = 0.3;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.id = static_cast<int>(i);

    marker.pose = pose_arr[i];
    markerArr.markers.push_back(marker);
  }
  marker_pub->publish(markerArr);
}

void PlaceBase::showBaseLocationsbyArmModel(std::vector<geometry_msgs::msg::Pose> po)
{
  RCLCPP_INFO(node_->get_logger(), "Showing Base Locations by Arm Model:");
//   kinematics::Kinematics k;

  auto imServer = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "robot_model", node_);

  std::vector<visualization_msgs::msg::InteractiveMarker> iMarkers;
  BasePoseJoint base_pose_joints;
  
  for (size_t i = 0; i < po.size(); i++)
  {
    for (size_t j = 0; j < GRASP_POSES_.size(); j++)
    {
      std::vector<double> joint_soln;
      int nsolns = 0;
      isIkSuccesswithTransformedBase(po[i], GRASP_POSES_[j], joint_soln, nsolns);
      std::pair<std::vector<double>, geometry_msgs::msg::Pose> myPair(joint_soln, po[i]);
      base_pose_joints.insert(myPair);
    }
  }
  
  mark_->makeArmMarker(base_pose_joints, iMarkers, show_ureach_models_);
  
  for (size_t i = 0; i < iMarkers.size(); i++)
  {
    imServer->insert(iMarkers[i]);
  }

  imServer->applyChanges();
  rclcpp::sleep_for(std::chrono::seconds(25));
}

void PlaceBase::showBaseLocationsbyRobotModel(std::vector<geometry_msgs::msg::Pose> po)
{
  RCLCPP_INFO(node_->get_logger(), "Showing Base Locations by Robot Model:");
//   kinematics::Kinematics k;
  
  auto imServer = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "robot_model", node_);

  std::vector<visualization_msgs::msg::InteractiveMarker> iMarkers;
  BasePoseJoint base_pose_joints;
  
  for (size_t i = 0; i < po.size(); i++)
  {
    geometry_msgs::msg::Pose base_pose_at_arm;
    transformFromRobotbaseToArmBase(po[i], base_pose_at_arm);
    
    for (size_t j = 0; j < GRASP_POSES_.size(); j++)
    {
      std::vector<double> joint_soln;
      int nsolns = 0;
      isIkSuccesswithTransformedBase(base_pose_at_arm, GRASP_POSES_[j], joint_soln, nsolns);
      std::pair<std::vector<double>, geometry_msgs::msg::Pose> myPair(joint_soln, po[i]);
      base_pose_joints.insert(myPair);
    }
  }

  mark_->makeRobotMarker(base_pose_joints, iMarkers, show_ureach_models_);
  
  for (size_t i = 0; i < iMarkers.size(); i++)
  {
    imServer->insert(iMarkers[i]);
  }

  imServer->applyChanges();
  rclcpp::sleep_for(std::chrono::seconds(25));
}

void PlaceBase::setReachabilityData(
  std::multimap<std::vector<double>, std::vector<double>> PoseCollection,
  std::multimap<std::vector<double>, double> SphereCollection,
  float resolution)
{
  // Merge PoseCollection into PoseColFilter (don't replace if incoming is empty)
  if (!PoseCollection.empty())
  {
    PoseColFilter.insert(PoseCollection.begin(), PoseCollection.end());
    RCLCPP_INFO(node_->get_logger(), "Added %lu poses to PoseColFilter (total: %lu)",
                PoseCollection.size(), PoseColFilter.size());
  }

  // Merge SphereCollection into SphereCol (don't replace if incoming is empty)
  if (!SphereCollection.empty())
  {
    SphereCol.insert(SphereCollection.begin(), SphereCollection.end());
    RCLCPP_INFO(node_->get_logger(), "Added %lu spheres to SphereCol (total: %lu)",
                SphereCollection.size(), SphereCol.size());
  }

  // Update resolution if provided (non-zero)
  if (resolution > 0.0f)
  {
    res = resolution;
    RCLCPP_INFO(node_->get_logger(), "Updated resolution: %f", res);
  }

  RCLCPP_INFO(node_->get_logger(), "Current total - Poses: %lu, Spheres: %lu, Resolution: %f",
              PoseColFilter.size(), SphereCol.size(), res);

  // ============================================================
  // NOTE: Reachability data is now passed to server via the action goal
  // when findbase() is called. We compute union map and high-score spheres
  // locally and send them in the goal message.
  // ============================================================

  RCLCPP_INFO(node_->get_logger(),
    "Reachability data stored locally (resolution: %f). Will be sent to server when computing base placement.",
    res);
}

void PlaceBase::ShowUnionMap(bool /* show_map */)
{
  RCLCPP_INFO(node_->get_logger(), "Showing Map:");
  
  if (sphereColor.size() == 0)
  {
    RCLCPP_INFO(node_->get_logger(), 
      "The union map has not been created yet. Please create the Union map by the Find Base button");
  }
  else
  {
    // change it for a point cloud
    // auto workspace_pub = node_->create_publisher<PlaceBase::WorkSpace>("reachability_map", 1);
    
    PlaceBase::WorkSpace ws;
    ws.header.stamp = node_->now();
    ws.header.frame_id = "base_link";
    ws.resolution = res;

    for (std::multimap<std::vector<double>, double>::iterator it = sphereColor.begin(); 
         it != sphereColor.end(); ++it)
    {
      PlaceBase::WsSphere wss;
      wss.point.x = it->first[0];
      wss.point.y = it->first[1];
      wss.point.z = it->first[2];
      wss.ri = (float)it->second;
      ws.ws_spheres.push_back(wss);
    }
    // workspace_pub->publish(ws);
  }
}

void PlaceBase::clearUnionMap(bool clear_map)
{
  if (!clear_map) {
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Clearing maps via service...");

  // Wait for service to be available
  if (!clear_maps_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "clear_maps service not available");
    // Clear local data anyway
    baseTrnsCol.clear();
    sphereColor.clear();
    highScoreSp.clear();
    robot_PoseColfilter.clear();
    GRASP_POSES_.clear();
    final_base_poses.clear();
    return;
  }

  // Call service
  auto request = std::make_shared<base_placement_interfaces::srv::ClearMaps::Request>();
  request->clear_union_map = true;
  request->clear_task_poses = true;

  auto result_future = clear_maps_client_->async_send_request(request);

  // Wait for result (with timeout)
  if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(5))
      == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    if (response->success) {
      RCLCPP_INFO(node_->get_logger(), "Maps cleared successfully: %s", response->message.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to clear maps: %s", response->message.c_str());
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call clear_maps service");
  }

  // Clear local data
  baseTrnsCol.clear();
  sphereColor.clear();
  highScoreSp.clear();
  robot_PoseColfilter.clear();
  GRASP_POSES_.clear();
  final_base_poses.clear();

  RCLCPP_INFO(node_->get_logger(), "Local maps cleared");
}

bool PlaceBase::isIkSuccesswithTransformedBase(const geometry_msgs::msg::Pose& base_pose,
                                              const geometry_msgs::msg::Pose& grasp_pose,
                                              std::vector<double>& joint_soln,
                                              int& numOfSolns){
  // Create the transformation inverse taking base_pose as world base
  // Tb_g = Tw_b-1 * Tw_g
  tf2::Transform base_pose_trans;
  base_pose_trans.setOrigin(tf2::Vector3(base_pose.position.x,
                                          base_pose.position.y,
                                          base_pose.position.z));
  base_pose_trans.setRotation(tf2::Quaternion(base_pose.orientation.x,
                                                base_pose.orientation.y,
                                                base_pose.orientation.z,
                                                base_pose.orientation.w));

  tf2::Transform grasp_pose_trans;
  grasp_pose_trans.setOrigin(tf2::Vector3(grasp_pose.position.x,
                                          grasp_pose.position.y,
                                          grasp_pose.position.z));
  grasp_pose_trans.setRotation(tf2::Quaternion(grasp_pose.orientation.x,
                                                grasp_pose.orientation.y,
                                                grasp_pose.orientation.z,
                                                grasp_pose.orientation.w));

  tf2::Transform base_grasp_pose_trans = base_pose_trans.inverse() * grasp_pose_trans;

  // Call the inverse kin service curobo
  auto request = std::make_shared<curobo_msgs::srv::Ik::Request>();

  // Convert tf2::Transform to geometry_msgs::msg::Pose
  // geometry_msgs::msg::Pose target_pose;
  request->pose.position.x = base_grasp_pose_trans.getOrigin().x();
  request->pose.position.y = base_grasp_pose_trans.getOrigin().y();
  request->pose.position.z = base_grasp_pose_trans.getOrigin().z();
  request->pose.orientation.x = base_grasp_pose_trans.getRotation().x();
  request->pose.orientation.y = base_grasp_pose_trans.getRotation().y();
  request->pose.orientation.z = base_grasp_pose_trans.getRotation().z();
  request->pose.orientation.w = base_grasp_pose_trans.getRotation().w();

  // request->poses.push_back(target_pose);
  auto result_future = this->client_ik->async_send_request(request);

  std::future_status status = result_future.wait_for(std::chrono::seconds(10)); // timeout to guarantee a graceful finish
  if (status == std::future_status::ready)
  {
      auto res = result_future.get();
      if (!res->success)
      {
          RCLCPP_ERROR(node_->get_logger(), "Ik batch failed");
          return false;
      }


      // Check if the first joint state is valid
      if (res->joint_states_valid.data)
      {
          // Extract joint positions from the first joint state
          joint_soln = res->joint_states.position;
          return true;
      }
      else{
      RCLCPP_WARN(node_->get_logger(), "No valid IK solution found");
      }


      return false;
  }
  else
  {
      RCLCPP_ERROR(node_->get_logger(), "Service call timed out");
      return false;
  }
  return false;
}

// ============================================================
// ACTION CLIENT CALLBACKS
// ============================================================

void PlaceBase::goalResponseCallback(const GoalHandleFindBase::SharedPtr& goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    Q_EMIT basePlacementProcessFinished();
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    goal_handle_ = goal_handle;
  }
}

void PlaceBase::feedbackCallback(
  GoalHandleFindBase::SharedPtr,
  const std::shared_ptr<const FindBaseAction::Feedback> feedback)
{
  RCLCPP_INFO(node_->get_logger(),
    "Feedback: phase='%s', iteration=%d/%d, progress=%.1f%%, best_score=%.2f",
    feedback->current_phase.c_str(),
    feedback->iteration,
    feedback->total_iterations,
    feedback->progress_percentage,
    feedback->current_best_score);

  // You can emit Qt signals here to update the GUI with progress
  // For example: Q_EMIT updateProgress(feedback->progress_percentage);
}

void PlaceBase::resultCallback(const GoalHandleFindBase::WrappedResult& result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Goal succeeded!");

      // Store the results
      final_base_poses.clear();
      for (const auto& pose : result.result->base_poses) {
        final_base_poses.push_back(pose);
      }

      score_ = result.result->best_score;

      RCLCPP_INFO(node_->get_logger(),
        "Received %zu base poses with best score: %.2f, computation time: %.3fs",
        final_base_poses.size(),
        result.result->best_score,
        result.result->computation_time_seconds);

      // Emit signals to update GUI
      Q_EMIT basePlacementProcessCompleted(score_);
      Q_EMIT basePlacementProcessFinished();

      // Visualize the results based on selected output type
      OuputputVizHandler(final_base_poses);
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted: %s", result.result->message.c_str());
      Q_EMIT basePlacementProcessFinished();
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
      Q_EMIT basePlacementProcessFinished();
      break;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      Q_EMIT basePlacementProcessFinished();
      break;
  }
}