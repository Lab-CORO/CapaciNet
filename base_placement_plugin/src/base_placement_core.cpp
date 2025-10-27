#include <base_placement_plugin/base_placement_core.h>
#include <rclcpp/rclcpp.hpp>
#include <utils.h>
#include <chrono>

// Static member initialization
const std::vector<std::string> BasePlacementCore::method_names_ = {
  "PCA",
  "GraspReachabilityScore",
  "IKSolutionScore",
  "VerticalRobotModel",
  "UserIntuition"
};

// ============================================================
// CONSTRUCTOR & DESTRUCTOR
// ============================================================

BasePlacementCore::BasePlacementCore(std::shared_ptr<rclcpp::Node> node)
  : node_(node)
  , selected_method_(Method::GraspReachabilityScore)
  , num_base_locations_(5)
  , num_high_score_spheres_(100)
  , resolution_(0.0)
  , best_score_(0.0)
{
  RCLCPP_INFO(node_->get_logger(), "BasePlacementCore initialized");
}

BasePlacementCore::~BasePlacementCore()
{
  RCLCPP_INFO(node_->get_logger(), "BasePlacementCore destroyed");
}

void BasePlacementCore::init()
{
  // Create callback group for IK service client
  client_cb_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive
  );

  // Create IK service client
  client_ik_ = node_->create_client<curobo_msgs::srv::Ik>(
    "compute_ik",
    rmw_qos_profile_services_default,
    client_cb_group_
  );

  RCLCPP_INFO(node_->get_logger(), "BasePlacementCore initialized with IK client");
}

// ============================================================
// CONFIGURATION METHODS
// ============================================================

void BasePlacementCore::setParameters(Method method, int num_base_locations, int num_high_score_spheres)
{
  selected_method_ = method;
  num_base_locations_ = num_base_locations;
  num_high_score_spheres_ = num_high_score_spheres;

  RCLCPP_INFO(node_->get_logger(),
    "Parameters updated: method=%s, base_locations=%d, high_score_spheres=%d",
    getMethodName(method).c_str(), num_base_locations, num_high_score_spheres);
}

void BasePlacementCore::setMethod(Method method)
{
  selected_method_ = method;
  RCLCPP_INFO(node_->get_logger(), "Method set to: %s", getMethodName(method).c_str());
}

void BasePlacementCore::setNumBaseLocations(int num)
{
  num_base_locations_ = num;
  RCLCPP_INFO(node_->get_logger(), "Number of base locations set to: %d", num);
}

void BasePlacementCore::setNumHighScoreSpheres(int num)
{
  num_high_score_spheres_ = num;
  RCLCPP_INFO(node_->get_logger(), "Number of high score spheres set to: %d", num);
}

void BasePlacementCore::setHighScoreSpheres(const std::vector<std::vector<double>>& spheres)
{
  high_score_sp_ = spheres;
  RCLCPP_INFO(node_->get_logger(),
    "Set %zu pre-computed high-score spheres from client", spheres.size());
}

void BasePlacementCore::setBaseTrnsCol(const std::multimap<std::vector<double>, std::vector<double>>& base_trns_col)
{
  base_trns_col_ = base_trns_col;
  RCLCPP_INFO(node_->get_logger(),
    "Set %zu union map poses from client", base_trns_col.size());
}

// ============================================================
// DATA LOADING METHODS
// ============================================================

bool BasePlacementCore::setReachabilityData(
  std::multimap<std::vector<double>, std::vector<double>> pose_collection,
  std::multimap<std::vector<double>, double> sphere_collection,
  float resolution)
{
  pose_col_filter_ = pose_collection;
  sphere_col_ = sphere_collection;
  resolution_ = resolution;

  RCLCPP_INFO(node_->get_logger(),
    "Reachability data loaded: %zu poses, %zu spheres, resolution=%.4f",
    pose_collection.size(), sphere_collection.size(), resolution);

  // Transform to robot base coordinates
  transformToRobotbase(pose_col_filter_, robot_pose_col_filter_);

  return true;
}

bool BasePlacementCore::loadReachabilityFromFile(
  const std::string& irm_file_path,
  const std::string& rm_file_path)
{
  RCLCPP_INFO(node_->get_logger(), "Loading reachability data from files...");
  RCLCPP_INFO(node_->get_logger(), "  IRM file: %s", irm_file_path.c_str());
  if (!rm_file_path.empty()) {
    RCLCPP_INFO(node_->get_logger(), "  RM file: %s", rm_file_path.c_str());
  }

  // Load IRM (Inverse Reachability Map) - contains sphere scores
  std::map<utils::QuantizedPoint3D, double> reachability_map;
  double resolution = 0.0;
  std::array<double, 3> voxel_grid_origin;
  std::array<int, 3> voxel_grid_sizes;

  bool irm_success = utils::loadFromHDF5(
      irm_file_path,
      reachability_map,
      resolution,
      voxel_grid_origin,
      voxel_grid_sizes,
      0  // group_id
  );

  if (!irm_success) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load IRM file");
    return false;
  }

  // Convert reachability map to sphere_col format
  sphere_col_.clear();
  for (const auto& entry : reachability_map) {
    const utils::QuantizedPoint3D& point = entry.first;
    double score = entry.second;

    std::vector<double> sphere_coord(3);
    sphere_coord[0] = point.x * resolution;
    sphere_coord[1] = point.y * resolution;
    sphere_coord[2] = point.z * resolution;

    sphere_col_.insert({sphere_coord, score});
  }

  resolution_ = static_cast<float>(resolution);

  RCLCPP_INFO(node_->get_logger(), "Loaded IRM: %zu reachable spheres with resolution %.3f m",
              sphere_col_.size(), resolution_);

  // Load RM (Master Reachability Map) - contains poses for each sphere
  if (!rm_file_path.empty()) {
    // Load poses from NPZ file
    std::vector<geometry_msgs::msg::Pose> poses;
    bool rm_success = utils::load_poses_from_file(rm_file_path, poses);

    if (rm_success && !poses.empty()) {
      RCLCPP_INFO(node_->get_logger(),
                  "Loaded %zu poses from RM file", poses.size());

      // Convert poses to pose_col_filter_ format
      sphere_discretization::SphereDiscretization sd;
      pose_col_filter_.clear();

      for (const auto& pose : poses) {
        // Use position as key (sphere position)
        std::vector<double> position(3);
        position[0] = pose.position.x;
        position[1] = pose.position.y;
        position[2] = pose.position.z;

        // Convert full pose to vector (with orientation)
        std::vector<double> pose_vec;
        sd.convertPoseToVector(pose, pose_vec);

        pose_col_filter_.insert({position, pose_vec});
      }

      RCLCPP_INFO(node_->get_logger(),
                  "Created pose collection with %zu entries", pose_col_filter_.size());
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Failed to load RM file or file is empty. Creating sample poses from IRM.");

      // Fallback: create sample poses from high-scoring spheres
      sphere_discretization::SphereDiscretization sd;
      pose_col_filter_.clear();

      int poses_created = 0;
      int max_poses = 1000;  // Limit to avoid too much data

      for (const auto& entry : sphere_col_) {
        const std::vector<double>& sphere_pos = entry.first;
        double score = entry.second;

        // Only use high-scoring spheres (score > 0.5) and limit total poses
        if (score > 0.5 && poses_created < max_poses) {
          // Create a sample pose at this sphere position
          geometry_msgs::msg::Pose pose;
          pose.position.x = sphere_pos[0];
          pose.position.y = sphere_pos[1];
          pose.position.z = sphere_pos[2];
          pose.orientation.w = 1.0;
          pose.orientation.x = 0.0;
          pose.orientation.y = 0.0;
          pose.orientation.z = 0.0;

          std::vector<double> pose_vec;
          sd.convertPoseToVector(pose, pose_vec);
          pose_col_filter_.insert({sphere_pos, pose_vec});
          poses_created++;
        }
      }

      RCLCPP_INFO(node_->get_logger(), "Created %d sample poses from high-score spheres",
                  poses_created);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Reachability data loaded successfully");
  return true;
}

// ============================================================
// POSE MANAGEMENT METHODS
// ============================================================

bool BasePlacementCore::addNamedPose(const std::string& name, const geometry_msgs::msg::Pose& pose)
{
  named_task_poses_[name] = pose;
  RCLCPP_INFO(node_->get_logger(), "Added named pose '%s'. Total poses: %zu",
    name.c_str(), named_task_poses_.size());
  return true;
}

bool BasePlacementCore::removeNamedPose(const std::string& name)
{
  auto it = named_task_poses_.find(name);
  if (it != named_task_poses_.end()) {
    named_task_poses_.erase(it);
    RCLCPP_INFO(node_->get_logger(), "Removed named pose '%s'. Total poses: %zu",
      name.c_str(), named_task_poses_.size());
    return true;
  }
  RCLCPP_WARN(node_->get_logger(), "Pose '%s' not found", name.c_str());
  return false;
}

void BasePlacementCore::clearTaskPoses()
{
  named_task_poses_.clear();
  RCLCPP_INFO(node_->get_logger(), "All task poses cleared");
}

std::vector<BasePlacementCore::NamedPose> BasePlacementCore::getTaskPoses() const
{
  std::vector<NamedPose> result;
  for (const auto& [name, pose] : named_task_poses_) {
    result.push_back({name, pose});
  }
  return result;
}

void BasePlacementCore::setTaskPoses(const std::vector<geometry_msgs::msg::Pose>& poses)
{
  named_task_poses_.clear();
  for (size_t i = 0; i < poses.size(); ++i) {
    std::string name = "pose_" + std::to_string(i);
    named_task_poses_[name] = poses[i];
  }
  RCLCPP_INFO(node_->get_logger(), "Set %zu task poses", poses.size());
}

void BasePlacementCore::setUserBasePoses(const std::vector<geometry_msgs::msg::Pose>& poses)
{
  user_base_poses_ = poses;
  RCLCPP_INFO(node_->get_logger(), "Set %zu user-defined base poses for UserIntuition method", poses.size());
}

// ============================================================
// COMPUTATION METHODS
// ============================================================

BasePlacementCore::ComputationResult BasePlacementCore::findBasePlacements(
  const std::vector<geometry_msgs::msg::Pose>& task_poses,
  FeedbackCallback feedback_callback)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  RCLCPP_INFO(node_->get_logger(), "Starting base placement computation with method: %s",
    getMethodName(selected_method_).c_str());

  ComputationResult result;

  // ========== COMPUTE UNION MAP AND HIGH-SCORE SPHERES IF NEEDED ==========
  // Check if we need to compute union map (not pre-computed by client)
  if (base_trns_col_.empty() || high_score_sp_.empty())
  {
    RCLCPP_INFO(node_->get_logger(),
                "Computing union map and high-score spheres from loaded reachability data...");

    if (pose_col_filter_.empty() || sphere_col_.empty())
    {
      result.success = false;
      result.message = "No reachability data loaded. Please load IRM/RM files first.";
      RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
      return result;
    }

    // Create sphere discretization helper
    sphere_discretization::SphereDiscretization sd;

    // Compute union map: associate task poses with reachability poses
    base_trns_col_.clear();
    if (selected_method_ == Method::VerticalRobotModel)
    {
      // VerticalRobotModel needs robot base transformation
      sd.associatePose(base_trns_col_, task_poses, robot_pose_col_filter_, resolution_);
      createSpheres(base_trns_col_, sphere_color_, high_score_sp_, true);
    }
    else
    {
      // Other methods use arm base directly
      sd.associatePose(base_trns_col_, task_poses, pose_col_filter_, resolution_);
      createSpheres(base_trns_col_, sphere_color_, high_score_sp_, false);
    }

    RCLCPP_INFO(node_->get_logger(),
                "Union map computed: %zu poses, %zu spheres, %zu high-score spheres",
                base_trns_col_.size(), sphere_color_.size(), high_score_sp_.size());

    // Validate we have enough data
    if (high_score_sp_.empty())
    {
      result.success = false;
      result.message = "No high-score spheres generated from union map computation.";
      RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
      return result;
    }
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(),
                "Using pre-computed union map: %zu poses, %zu high-score spheres",
                base_trns_col_.size(), high_score_sp_.size());
  }

  // Dispatch to the appropriate algorithm
  switch (selected_method_) {
    case Method::PCA:
      result = findBaseByPCA(task_poses, feedback_callback);
      break;
    case Method::GraspReachabilityScore:
      result = findBaseByGraspReachabilityScore(task_poses, feedback_callback);
      break;
    case Method::IKSolutionScore:
      result = findBaseByIKSolutionScore(task_poses, feedback_callback);
      break;
    case Method::VerticalRobotModel:
      result = findBaseByVerticalRobotModel(task_poses, feedback_callback);
      break;
    case Method::UserIntuition:
      result = findBaseByUserIntuition(task_poses, feedback_callback);
      break;
    default:
      result.success = false;
      result.message = "Unknown method";
      return result;
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end_time - start_time;
  result.computation_time_seconds = elapsed.count();

  // Store results
  if (result.success) {
    computed_base_poses_ = result.base_poses;
    computed_scores_ = result.scores;
    best_score_ = result.best_score;
    if (result.best_index >= 0 && result.best_index < static_cast<int>(result.base_poses.size())) {
      best_pose_ = result.base_poses[result.best_index];
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Computation finished in %.3f seconds. Success: %s",
    result.computation_time_seconds, result.success ? "true" : "false");

  return result;
}

BasePlacementCore::WorkSpace BasePlacementCore::getUnionMap(bool compute_from_current_poses)
{
  WorkSpace workspace;
  workspace.header.stamp = node_->now();
  workspace.header.frame_id = "map";
  workspace.resolution = resolution_;

  // If compute_from_current_poses is true, recompute union map from current task poses
  if (compute_from_current_poses && !named_task_poses_.empty())
  {
    RCLCPP_INFO(node_->get_logger(),
                "Computing union map from %zu current task poses...",
                named_task_poses_.size());

    // Convert named poses to vector
    std::vector<geometry_msgs::msg::Pose> task_poses;
    for (const auto& [name, pose] : named_task_poses_)
    {
      task_poses.push_back(pose);
    }

    // Create sphere discretization helper
    sphere_discretization::SphereDiscretization sd;

    // Compute union map
    base_trns_col_.clear();
    if (selected_method_ == Method::VerticalRobotModel)
    {
      sd.associatePose(base_trns_col_, task_poses, robot_pose_col_filter_, resolution_);
      createSpheres(base_trns_col_, sphere_color_, high_score_sp_, true);
    }
    else
    {
      sd.associatePose(base_trns_col_, task_poses, pose_col_filter_, resolution_);
      createSpheres(base_trns_col_, sphere_color_, high_score_sp_, false);
    }

    RCLCPP_INFO(node_->get_logger(),
                "Union map computed: %zu poses, %zu spheres, %zu high-score spheres",
                base_trns_col_.size(), sphere_color_.size(), high_score_sp_.size());
  }

  // Convert base_trns_col_ and sphere_color_ to WorkSpace format
  if (!base_trns_col_.empty())
  {
    // Group poses by sphere position (key in multimap)
    std::map<std::vector<double>, std::vector<geometry_msgs::msg::Pose>> grouped_poses;

    for (const auto& [sphere_pos, pose_vec] : base_trns_col_)
    {
      geometry_msgs::msg::Pose pose;
      if (pose_vec.size() >= 7)
      {
        pose.position.x = pose_vec[0];
        pose.position.y = pose_vec[1];
        pose.position.z = pose_vec[2];
        pose.orientation.x = pose_vec[3];
        pose.orientation.y = pose_vec[4];
        pose.orientation.z = pose_vec[5];
        pose.orientation.w = pose_vec[6];
        grouped_poses[sphere_pos].push_back(pose);
      }
    }

    // Create WsSphere for each unique sphere position
    for (const auto& [sphere_pos, poses] : grouped_poses)
    {
      WsSphere wss;
      wss.header.stamp = node_->now();
      wss.header.frame_id = "map";

      // Set sphere center from position key
      wss.point.x = sphere_pos[0];
      wss.point.y = sphere_pos[1];
      wss.point.z = sphere_pos[2];

      // Get score from sphere_color_ if available
      auto score_it = sphere_color_.find(sphere_pos);
      if (score_it != sphere_color_.end())
      {
        wss.ri = static_cast<float>(score_it->second / 100.0 * resolution_);
      }
      else
      {
        wss.ri = resolution_ * 0.5f; // Default radius
      }

      wss.poses = poses;
      workspace.ws_spheres.push_back(wss);
    }

    RCLCPP_INFO(node_->get_logger(),
                "Workspace created with %zu spheres", workspace.ws_spheres.size());
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(),
                "Union map is empty. Load reachability data and add task poses first.");
  }

  return workspace;
}

// ============================================================
// RESULT RETRIEVAL METHODS
// ============================================================

std::vector<geometry_msgs::msg::Pose> BasePlacementCore::getComputedBasePoses() const
{
  return computed_base_poses_;
}

std::vector<double> BasePlacementCore::getComputedScores() const
{
  return computed_scores_;
}

double BasePlacementCore::getBestScore() const
{
  return best_score_;
}

geometry_msgs::msg::Pose BasePlacementCore::getBestPose() const
{
  return best_pose_;
}

float BasePlacementCore::getResolution() const
{
  return resolution_;
}

const std::multimap<std::vector<double>, double>& BasePlacementCore::getSphereCollection() const
{
  return sphere_col_;
}

// ============================================================
// UTILITY METHODS
// ============================================================

void BasePlacementCore::clearAllData()
{
  clearReachabilityData();
  clearUnionMap();
  clearTaskPoses();
  computed_base_poses_.clear();
  computed_scores_.clear();
  best_score_ = 0.0;
  RCLCPP_INFO(node_->get_logger(), "All data cleared");
}

void BasePlacementCore::clearUnionMap()
{
  base_trns_col_.clear();
  sphere_color_.clear();
  high_score_sp_.clear();
  RCLCPP_INFO(node_->get_logger(), "Union map cleared");
}

void BasePlacementCore::clearReachabilityData()
{
  pose_col_filter_.clear();
  sphere_col_.clear();
  robot_pose_col_filter_.clear();
  resolution_ = 0.0;
  RCLCPP_INFO(node_->get_logger(), "Reachability data cleared");
}

std::vector<std::string> BasePlacementCore::getMethodNames()
{
  return method_names_;
}

std::string BasePlacementCore::getMethodName(Method method)
{
  int index = static_cast<int>(method);
  if (index >= 0 && index < static_cast<int>(method_names_.size())) {
    return method_names_[index];
  }
  return "Unknown";
}

// ============================================================
// ALGORITHM IMPLEMENTATIONS (Placeholder - to be implemented)
// ============================================================

BasePlacementCore::ComputationResult BasePlacementCore::findBaseByPCA(
  const std::vector<geometry_msgs::msg::Pose>& task_poses,
  FeedbackCallback feedback_callback)
{
  auto start_time = std::chrono::steady_clock::now();
  ComputationResult result;

  RCLCPP_INFO(node_->get_logger(), "Finding optimal base pose by PCA.");

  if (feedback_callback) {
    feedback_callback("PCA", 0, num_base_locations_, 0.0,
                     "Starting PCA method", 0, 0.0);
  }

  // Validation: check if high_score_sp_ has enough elements
  if (high_score_sp_.size() < static_cast<size_t>(num_base_locations_)) {
    result.success = false;
    result.message = "Not enough high score spheres! Required: " +
                     std::to_string(num_base_locations_) +
                     ", Available: " + std::to_string(high_score_sp_.size());
    RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  // Validation: check if base_trns_col_ is not empty
  if (base_trns_col_.empty()) {
    result.success = false;
    result.message = "base_trns_col_ is empty! Cannot proceed with PCA.";
    RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  sphere_discretization::SphereDiscretization sd;
  std::vector<geometry_msgs::msg::Pose> pose_scores;
  WorkSpace ws;

  // Build WorkSpace with spheres and their poses
  for (int i = 0; i < num_base_locations_; ++i) {
    if (feedback_callback) {
      double progress = (static_cast<double>(i) / num_base_locations_) * 50.0;
      feedback_callback("PCA", i, num_base_locations_, progress,
                       "Building workspace spheres", i, 0.0);
    }

    // Additional safety check
    if (high_score_sp_[i].size() < 3) {
      RCLCPP_ERROR(node_->get_logger(),
        "high_score_sp_[%d] doesn't have 3 elements! Size: %zu",
        i, high_score_sp_[i].size());
      continue;
    }

    WsSphere wss;
    wss.point.x = high_score_sp_[i][0];
    wss.point.y = high_score_sp_[i][1];
    wss.point.z = high_score_sp_[i][2];

    std::vector<double> basePose;
    basePose.push_back(high_score_sp_[i][0]);
    basePose.push_back(high_score_sp_[i][1]);
    basePose.push_back(high_score_sp_[i][2]);

    std::multimap<std::vector<double>, std::vector<double>>::iterator it;
    for (it = base_trns_col_.lower_bound(basePose);
         it != base_trns_col_.upper_bound(basePose); ++it) {
      geometry_msgs::msg::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      wss.poses.push_back(pp);
    }

    // Only add sphere if it has poses
    if (!wss.poses.empty()) {
      ws.ws_spheres.push_back(wss);
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "No poses found for sphere %d at position [%f, %f, %f]",
        i, wss.point.x, wss.point.y, wss.point.z);
    }
  }

  // Check if we have any valid spheres
  if (ws.ws_spheres.empty()) {
    result.success = false;
    result.message = "No valid spheres with poses found! Cannot proceed.";
    RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  if (feedback_callback) {
    feedback_callback("PCA", num_base_locations_, num_base_locations_ * 2, 50.0,
                     "Computing PCA for each sphere",
                     static_cast<int>(ws.ws_spheres.size()), 0.0);
  }

  // Compute optimal pose for each sphere using PCA
  for (size_t i = 0; i < ws.ws_spheres.size(); ++i) {
    if (feedback_callback) {
      double progress = 50.0 + (static_cast<double>(i) / ws.ws_spheres.size()) * 40.0;
      feedback_callback("PCA", static_cast<int>(i) + num_base_locations_,
                       num_base_locations_ * 2, progress,
                       "Computing PCA for sphere " + std::to_string(i),
                       static_cast<int>(i), 0.0);
    }

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

  if (feedback_callback) {
    feedback_callback("PCA", num_base_locations_ * 2, num_base_locations_ * 2, 90.0,
                     "Calculating final scores",
                     static_cast<int>(pose_scores.size()), 0.0);
  }

  // Convert task_poses to vector for scoring
  std::vector<geometry_msgs::msg::Pose> grasp_poses = task_poses;

  // Calculate score
  double score = calculateScoreForArmBase(grasp_poses, pose_scores);

  if (feedback_callback) {
    feedback_callback("PCA", num_base_locations_ * 2, num_base_locations_ * 2, 100.0,
                     "PCA method completed",
                     static_cast<int>(pose_scores.size()), score);
  }

  // Store results
  result.success = true;
  result.base_poses = pose_scores;
  result.scores.push_back(score);
  result.best_score = score;
  result.best_index = 0;
  result.message = "PCA method completed successfully";

  // Update internal state
  computed_base_poses_ = pose_scores;
  computed_scores_ = result.scores;
  best_score_ = score;

  auto end_time = std::chrono::steady_clock::now();
  result.computation_time_seconds =
    std::chrono::duration<double>(end_time - start_time).count();

  RCLCPP_INFO(node_->get_logger(),
    "PCA completed: %zu poses, score=%.2f, time=%.3fs",
    pose_scores.size(), score, result.computation_time_seconds);

  return result;
}

BasePlacementCore::ComputationResult BasePlacementCore::findBaseByGraspReachabilityScore(
  const std::vector<geometry_msgs::msg::Pose>& task_poses,
  FeedbackCallback feedback_callback)
{
  auto start_time = std::chrono::steady_clock::now();
  ComputationResult result;

  RCLCPP_INFO(node_->get_logger(), "Finding optimal base pose by GraspReachabilityScore.");

  if (feedback_callback) {
    feedback_callback("GraspReachabilityScore", 0, num_base_locations_, 0.0,
                     "Starting Grasp Reachability Score method", 0, 0.0);
  }

  // Validation checks
  if (high_score_sp_.size() < static_cast<size_t>(num_base_locations_)) {
    result.success = false;
    result.message = "Not enough high score spheres for GraspReachabilityScore";
    RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  if (base_trns_col_.empty()) {
    result.success = false;
    result.message = "base_trns_col_ is empty! Cannot proceed.";
    RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  sphere_discretization::SphereDiscretization sd;
  std::vector<geometry_msgs::msg::Pose> pose_scores;
  int numofSp = num_base_locations_;

  // Convert task_poses to vector
  std::vector<geometry_msgs::msg::Pose> grasp_poses = task_poses;

  // For each high score sphere
  for (int i = 0; i < numofSp; i++) {
    if (feedback_callback) {
      double progress = (static_cast<double>(i) / numofSp) * 100.0;
      feedback_callback("GraspReachabilityScore", i, numofSp, progress,
                       "Processing sphere " + std::to_string(i),
                       i, 0.0);
    }

    std::vector<geometry_msgs::msg::Pose> probBasePoses;
    std::multimap<std::vector<double>, std::vector<double>>::iterator it;

    // Get all probable base poses for this sphere
    for (it = base_trns_col_.lower_bound(high_score_sp_[i]);
         it != base_trns_col_.upper_bound(high_score_sp_[i]); ++it) {
      geometry_msgs::msg::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      probBasePoses.push_back(pp);
    }

    // Score each probable base pose by counting IK hits
    std::map<int, geometry_msgs::msg::Pose> basePoseWithHits;
    for (size_t j = 0; j < probBasePoses.size(); j++) {
      int numofHits = 0;
      for (size_t j1 = 0; j1 < grasp_poses.size(); j1++) {
        int nsolns = 0;
        std::vector<double> joint_solns;
        bool ik_success = isIkSuccesswithTransformedBase(probBasePoses[j], grasp_poses[j1],
                                                          joint_solns, nsolns);
        if (ik_success) {
          numofHits += nsolns;
        }
      }
      basePoseWithHits.insert(std::make_pair(numofHits, probBasePoses[j]));
    }

    // Select the base pose with the highest hits
    if (!basePoseWithHits.empty()) {
      std::map<int, geometry_msgs::msg::Pose>::iterator itr;
      itr = basePoseWithHits.end();
      --itr;
      pose_scores.push_back(itr->second);
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "No valid base poses found for sphere %d", i);
    }
  }

  if (pose_scores.empty()) {
    result.success = false;
    result.message = "No valid base poses found";
    RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  if (feedback_callback) {
    feedback_callback("GraspReachabilityScore", numofSp, numofSp, 95.0,
                     "Calculating final scores",
                     static_cast<int>(pose_scores.size()), 0.0);
  }

  // Calculate final score
  double score = calculateScoreForArmBase(grasp_poses, pose_scores);

  if (feedback_callback) {
    feedback_callback("GraspReachabilityScore", numofSp, numofSp, 100.0,
                     "Grasp Reachability Score method completed",
                     static_cast<int>(pose_scores.size()), score);
  }

  // Store results
  result.success = true;
  result.base_poses = pose_scores;
  result.scores.push_back(score);
  result.best_score = score;
  result.best_index = 0;
  result.message = "GraspReachabilityScore method completed successfully";

  // Update internal state
  computed_base_poses_ = pose_scores;
  computed_scores_ = result.scores;
  best_score_ = score;

  auto end_time = std::chrono::steady_clock::now();
  result.computation_time_seconds =
    std::chrono::duration<double>(end_time - start_time).count();

  RCLCPP_INFO(node_->get_logger(),
    "GraspReachabilityScore completed: %zu poses, score=%.2f, time=%.3fs",
    pose_scores.size(), score, result.computation_time_seconds);

  return result;
}

BasePlacementCore::ComputationResult BasePlacementCore::findBaseByIKSolutionScore(
  const std::vector<geometry_msgs::msg::Pose>& task_poses,
  FeedbackCallback feedback_callback)
{
  auto start_time = std::chrono::steady_clock::now();
  ComputationResult result;

  RCLCPP_INFO(node_->get_logger(), "Finding optimal base pose by IKSolutionScore.");

  if (feedback_callback) {
    feedback_callback("IKSolutionScore", 0, num_base_locations_, 0.0,
                     "Starting IK Solution Score method", 0, 0.0);
  }

  // Validation checks
  if (high_score_sp_.size() < static_cast<size_t>(num_base_locations_)) {
    result.success = false;
    result.message = "Not enough high score spheres for IKSolutionScore";
    RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  if (base_trns_col_.empty()) {
    result.success = false;
    result.message = "base_trns_col_ is empty! Cannot proceed.";
    RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  sphere_discretization::SphereDiscretization sd;
  std::vector<geometry_msgs::msg::Pose> pose_scores;
  int numofSp = num_base_locations_;

  // Convert task_poses to vector
  std::vector<geometry_msgs::msg::Pose> grasp_poses = task_poses;

  // Calculate max and min possible solutions for scoring normalization
  int max_solns = static_cast<int>(grasp_poses.size()) * 8;
  int min_solns = 0;

  // For each high score sphere
  for (int i = 0; i < numofSp; i++) {
    if (feedback_callback) {
      double progress = (static_cast<double>(i) / numofSp) * 100.0;
      feedback_callback("IKSolutionScore", i, numofSp, progress,
                       "Processing sphere " + std::to_string(i),
                       i, 0.0);
    }

    std::vector<geometry_msgs::msg::Pose> probBasePoses;
    std::multimap<std::vector<double>, std::vector<double>>::iterator it;

    // Get all probable base poses for this sphere
    for (it = base_trns_col_.lower_bound(high_score_sp_[i]);
         it != base_trns_col_.upper_bound(high_score_sp_[i]); ++it) {
      geometry_msgs::msg::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      probBasePoses.push_back(pp);
    }

    // Score each probable base pose by counting IK solutions and normalizing
    std::map<double, geometry_msgs::msg::Pose> basePoseWithHits;
    for (size_t j = 0; j < probBasePoses.size(); j++) {
      int numofHits = 0;
      int solns = 0;
      for (size_t j1 = 0; j1 < grasp_poses.size(); j1++) {
        int nsolns = 0;
        std::vector<double> joint_solns;
        bool ik_success = isIkSuccesswithTransformedBase(probBasePoses[j], grasp_poses[j1],
                                                          joint_solns, nsolns);
        if (ik_success) {
          numofHits++;
        }
        solns += nsolns;
      }
      // Normalized score between 0 and 1
      double basePlaceScore = (double(solns) - double(min_solns)) /
                              (double(max_solns) - double(min_solns));
      basePoseWithHits.insert(std::make_pair(basePlaceScore, probBasePoses[j]));
    }

    // Select the base pose with the highest score
    if (!basePoseWithHits.empty()) {
      std::map<double, geometry_msgs::msg::Pose>::iterator itr;
      itr = basePoseWithHits.end();
      --itr;
      pose_scores.push_back(itr->second);
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "No valid base poses found for sphere %d", i);
    }
  }

  if (pose_scores.empty()) {
    result.success = false;
    result.message = "No valid base poses found";
    RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  if (feedback_callback) {
    feedback_callback("IKSolutionScore", numofSp, numofSp, 95.0,
                     "Calculating final scores",
                     static_cast<int>(pose_scores.size()), 0.0);
  }

  // Calculate final score
  double score = calculateScoreForArmBase(grasp_poses, pose_scores);

  if (feedback_callback) {
    feedback_callback("IKSolutionScore", numofSp, numofSp, 100.0,
                     "IK Solution Score method completed",
                     static_cast<int>(pose_scores.size()), score);
  }

  // Store results
  result.success = true;
  result.base_poses = pose_scores;
  result.scores.push_back(score);
  result.best_score = score;
  result.best_index = 0;
  result.message = "IKSolutionScore method completed successfully";

  // Update internal state
  computed_base_poses_ = pose_scores;
  computed_scores_ = result.scores;
  best_score_ = score;

  auto end_time = std::chrono::steady_clock::now();
  result.computation_time_seconds =
    std::chrono::duration<double>(end_time - start_time).count();

  RCLCPP_INFO(node_->get_logger(),
    "IKSolutionScore completed: %zu poses, score=%.2f, time=%.3fs",
    pose_scores.size(), score, result.computation_time_seconds);

  return result;
}

BasePlacementCore::ComputationResult BasePlacementCore::findBaseByVerticalRobotModel(
  const std::vector<geometry_msgs::msg::Pose>& task_poses,
  FeedbackCallback feedback_callback)
{
  auto start_time = std::chrono::steady_clock::now();
  ComputationResult result;

  RCLCPP_INFO(node_->get_logger(), "Finding optimal ROBOT base pose by Vertical robot model.");

  if (feedback_callback) {
    feedback_callback("VerticalRobotModel", 0, 1, 0.0,
                     "Starting vertical robot model method", 0, 0.0);
  }

  // Check if high_score_sp_ has data
  if (high_score_sp_.empty()) {
    result.success = false;
    result.message = "No high score spheres available. Run reachability computation first.";
    RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  std::vector<geometry_msgs::msg::Pose> base_poses;
  std::vector<geometry_msgs::msg::Pose> base_poses_user;

  int num_of_desired_sp = num_high_score_spheres_ + 50;
  int num_of_sp = static_cast<int>(high_score_sp_.size());

  if (num_base_locations_ > num_of_sp) {
    RCLCPP_ERROR(node_->get_logger(),
      "Desired base locations (%d) are too high. Available: %d. Please reduce it.",
      num_base_locations_, num_of_sp);
    result.success = false;
    result.message = "Desired base locations exceed available spheres";
    return result;
  }

  if (feedback_callback) {
    feedback_callback("VerticalRobotModel", 0, num_of_desired_sp, 25.0,
                     "Creating base poses from high-score spheres",
                     0, 0.0);
  }

  // Create base poses from high score spheres
  for (int i = 0; i < num_of_desired_sp && i < num_of_sp; i += 4) {
    geometry_msgs::msg::Pose prob_base_pose;
    prob_base_pose.position.x = high_score_sp_[i][0];
    prob_base_pose.position.y = high_score_sp_[i][1];
    prob_base_pose.position.z = high_score_sp_[i][2];
    prob_base_pose.orientation.x = 0;
    prob_base_pose.orientation.y = 0;
    prob_base_pose.orientation.z = 0;
    prob_base_pose.orientation.w = 1;
    base_poses.push_back(prob_base_pose);
  }

  if (feedback_callback) {
    feedback_callback("VerticalRobotModel", 1, 2, 50.0,
                     "Selecting top base locations",
                     static_cast<int>(base_poses.size()), 0.0);
  }

  // Select the top BASE_LOC_SIZE_ poses
  for (int i = 0; i < num_base_locations_ && i < static_cast<int>(base_poses.size()); ++i) {
    base_poses_user.push_back(base_poses[i]);
  }

  if (feedback_callback) {
    feedback_callback("VerticalRobotModel", 2, 3, 75.0,
                     "Calculating scores for selected base poses",
                     static_cast<int>(base_poses_user.size()), 0.0);
  }

  // Convert task_poses to vector for scoring
  std::vector<geometry_msgs::msg::Pose> grasp_poses = task_poses;

  // Calculate score
  double score = calculateScoreForRobotBase(grasp_poses, base_poses_user);

  if (feedback_callback) {
    feedback_callback("VerticalRobotModel", 3, 3, 100.0,
                     "Vertical robot model method completed",
                     static_cast<int>(base_poses_user.size()), score);
  }

  // Store results
  result.success = true;
  result.base_poses = base_poses_user;
  result.scores.push_back(score);
  result.best_score = score;
  result.best_index = 0;
  result.message = "VerticalRobotModel method completed successfully";

  // Update internal state
  computed_base_poses_ = base_poses_user;
  computed_scores_ = result.scores;
  best_score_ = score;

  auto end_time = std::chrono::steady_clock::now();
  result.computation_time_seconds =
    std::chrono::duration<double>(end_time - start_time).count();

  RCLCPP_INFO(node_->get_logger(),
    "VerticalRobotModel completed: %zu poses, score=%.2f, time=%.3fs",
    base_poses_user.size(), score, result.computation_time_seconds);

  return result;
}

BasePlacementCore::ComputationResult BasePlacementCore::findBaseByUserIntuition(
  const std::vector<geometry_msgs::msg::Pose>& task_poses,
  FeedbackCallback feedback_callback)
{
  auto start_time = std::chrono::steady_clock::now();
  ComputationResult result;

  RCLCPP_INFO(node_->get_logger(), "Finding optimal base pose by user intuition.");

  if (feedback_callback) {
    feedback_callback("UserIntuition", 0, 1, 0.0,
                     "Starting user intuition method", 0, 0.0);
  }

  // Check if user has defined base poses
  if (user_base_poses_.empty()) {
    result.success = false;
    result.message = "No user-defined base poses available. Use setUserBasePoses() first.";
    RCLCPP_ERROR(node_->get_logger(), "%s", result.message.c_str());
    return result;
  }

  // Convert task_poses to vector (it's already a vector, just copy for consistency)
  std::vector<geometry_msgs::msg::Pose> grasp_poses = task_poses;

  if (feedback_callback) {
    feedback_callback("UserIntuition", 0, 1, 50.0,
                     "Calculating score for user-defined base poses", 0, 0.0);
  }

  // Calculate score for user-defined base poses
  double score = calculateScoreForRobotBase(grasp_poses, user_base_poses_);

  if (feedback_callback) {
    feedback_callback("UserIntuition", 1, 1, 100.0,
                     "User intuition method completed",
                     static_cast<int>(user_base_poses_.size()), score);
  }

  // Store results
  result.success = true;
  result.base_poses = user_base_poses_;
  result.scores.push_back(score);
  result.best_score = score;
  result.best_index = 0;
  result.message = "UserIntuition method completed successfully";

  // Update internal state
  computed_base_poses_ = user_base_poses_;
  computed_scores_ = result.scores;
  best_score_ = score;

  auto end_time = std::chrono::steady_clock::now();
  result.computation_time_seconds =
    std::chrono::duration<double>(end_time - start_time).count();

  RCLCPP_INFO(node_->get_logger(),
    "UserIntuition completed: score=%.2f, time=%.3fs",
    score, result.computation_time_seconds);

  return result;
}

// ============================================================
// HELPER METHODS (Placeholder - to be implemented)
// ============================================================

void BasePlacementCore::transformToRobotbase(
  std::multimap<std::vector<double>, std::vector<double>> armBasePoses,
  std::multimap<std::vector<double>, std::vector<double>>& robotBasePoses)
{
  RCLCPP_WARN_ONCE(node_->get_logger(),
    "transformToRobotbase() using identity transform - implement with robot model");

  Eigen::Affine3d arm_to_root = Eigen::Affine3d::Identity();

  sphere_discretization::SphereDiscretization sd;
  for (auto it = armBasePoses.begin(); it != armBasePoses.end(); ++it)
  {
    geometry_msgs::msg::Pose arm_base_pose;
    sd.convertVectorToPose(it->second, arm_base_pose);

    geometry_msgs::msg::Transform arm_transform;
    arm_transform.translation.x = arm_base_pose.position.x;
    arm_transform.translation.y = arm_base_pose.position.y;
    arm_transform.translation.z = arm_base_pose.position.z;
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

void BasePlacementCore::transformFromRobotbaseToArmBase(
  const geometry_msgs::msg::Pose& base_pose,
  geometry_msgs::msg::Pose& arm_base_pose)
{
  RCLCPP_WARN_ONCE(node_->get_logger(),
    "transformFromRobotbaseToArmBase() using identity transform - implement with robot model");

  Eigen::Affine3d trans_to_arm_parent = Eigen::Affine3d::Identity();

  geometry_msgs::msg::Transform base_transform;
  base_transform.translation.x = base_pose.position.x;
  base_transform.translation.y = base_pose.position.y;
  base_transform.translation.z = base_pose.position.z;
  base_transform.rotation = base_pose.orientation;

  Eigen::Affine3d base_pose_tf = tf2::transformToEigen(base_transform);
  arm_base_pose = tf2::toMsg(base_pose_tf * trans_to_arm_parent);
}

void BasePlacementCore::createSpheres(
  std::multimap<std::vector<double>, std::vector<double>> basePoses,
  std::map<std::vector<double>, double>& spColor,
  std::vector<std::vector<double>>& highScoredSp,
  bool reduce_D)
{
  std::vector<int> poseCount;
  poseCount.reserve(basePoses.size());

  for (auto it = basePoses.begin(); it != basePoses.end(); ++it)
  {
    int num = basePoses.count(it->first);
    poseCount.push_back(num);
  }

  auto max_it = std::max_element(poseCount.begin(), poseCount.end());
  auto min_it = std::min_element(poseCount.begin(), poseCount.end());
  int max_number = (max_it != poseCount.end()) ? *max_it : 0;
  int min_number = (min_it != poseCount.end()) ? *min_it : 0;

  if (reduce_D)
  {
    // For vertical robot model - check Z coordinate
    std::vector<geometry_msgs::msg::Pose> task_poses_copy;
    for (const auto& [name, pose] : named_task_poses_)
    {
      task_poses_copy.push_back(pose);
    }

    for (auto it = basePoses.begin(); it != basePoses.end(); ++it)
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
        for (const auto& grasp_pose : task_poses_copy)
        {
          std::vector<double> joint_soln;
          int nsolns = 0;
          isIkSuccesswithTransformedBase(base_pose_at_arm, grasp_pose, joint_soln, nsolns);
          num_of_solns += nsolns;
        }

        float d = (task_poses_copy.size() > 0) ?
          (float(num_of_solns) / float(task_poses_copy.size() * 8)) * 100 : 0.0f;
        spColor.insert(std::pair<std::vector<double>, double>(it->first, double(d)));
      }
    }
  }
  else
  {
    for (auto it = basePoses.begin(); it != basePoses.end(); ++it)
    {
      if (max_number > min_number)
      {
        float d = ((float(basePoses.count(it->first)) - min_number) / (max_number - min_number)) * 100;
        if (d > 1)
        {
          spColor.insert(std::pair<std::vector<double>, double>(it->first, double(d)));
        }
      }
    }
  }

  // Sort by score
  std::multiset<std::pair<double, std::vector<double>>> scoreWithSp;
  for (auto it = spColor.begin(); it != spColor.end(); ++it)
  {
    scoreWithSp.insert(std::pair<double, std::vector<double>>(it->second, it->first));
  }

  for (auto it = scoreWithSp.rbegin(); it != scoreWithSp.rend(); ++it)
  {
    highScoredSp.push_back(it->second);
  }
}

double BasePlacementCore::calculateScoreForRobotBase(
  std::vector<geometry_msgs::msg::Pose>& grasp_poses,
  std::vector<geometry_msgs::msg::Pose>& base_poses)
{
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

double BasePlacementCore::calculateScoreForArmBase(
  std::vector<geometry_msgs::msg::Pose>& grasp_poses,
  std::vector<geometry_msgs::msg::Pose>& base_poses)
{
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

bool BasePlacementCore::isIkSuccesswithTransformedBase(
  const geometry_msgs::msg::Pose& base_pose,
  const geometry_msgs::msg::Pose& grasp_pose,
  std::vector<double>& joint_soln,
  int& numOfSolns)
{
  // Create the transformation: Tb_g = Tw_b^-1 * Tw_g
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

  // Call IK service
  auto request = std::make_shared<curobo_msgs::srv::Ik::Request>();
  request->pose.position.x = base_grasp_pose_trans.getOrigin().x();
  request->pose.position.y = base_grasp_pose_trans.getOrigin().y();
  request->pose.position.z = base_grasp_pose_trans.getOrigin().z();
  request->pose.orientation.x = base_grasp_pose_trans.getRotation().x();
  request->pose.orientation.y = base_grasp_pose_trans.getRotation().y();
  request->pose.orientation.z = base_grasp_pose_trans.getRotation().z();
  request->pose.orientation.w = base_grasp_pose_trans.getRotation().w();

  auto result_future = client_ik_->async_send_request(request);

  std::future_status status = result_future.wait_for(std::chrono::seconds(10));
  if (status == std::future_status::ready)
  {
    auto res = result_future.get();
    if (!res->success)
    {
      return false;
    }

    if (res->joint_states_valid.data)
    {
      joint_soln = res->joint_states.position;
      numOfSolns = 1;  // CuRobo returns 1 solution
      return true;
    }
  }

  return false;
}
