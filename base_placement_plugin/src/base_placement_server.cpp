#include <base_placement_plugin/base_placement_server.h>
#include <functional>

using namespace std::placeholders;

BasePlacementServer::BasePlacementServer(const rclcpp::NodeOptions& options)
  : Node("base_placement_server", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing BasePlacementServer");

  // Note: Cannot use shared_from_this() in constructor
  // Will create core after construction is complete

  // Create action server
  action_server_ = rclcpp_action::create_server<FindBase>(
    this,
    "find_base",
    std::bind(&BasePlacementServer::handle_goal, this, _1, _2),
    std::bind(&BasePlacementServer::handle_cancel, this, _1),
    std::bind(&BasePlacementServer::handle_accepted, this, _1)
  );

  // Create service servers
  srv_update_reachability_ = this->create_service<base_placement_interfaces::srv::UpdateReachabilityMap>(
    "update_reachability_map",
    std::bind(&BasePlacementServer::handle_update_reachability_map, this, _1, _2)
  );

  srv_get_union_map_ = this->create_service<base_placement_interfaces::srv::GetUnionMap>(
    "get_union_map",
    std::bind(&BasePlacementServer::handle_get_union_map, this, _1, _2)
  );

  srv_update_parameters_ = this->create_service<base_placement_interfaces::srv::UpdateParameters>(
    "update_parameters",
    std::bind(&BasePlacementServer::handle_update_parameters, this, _1, _2)
  );

  srv_add_named_pose_ = this->create_service<base_placement_interfaces::srv::AddNamedPose>(
    "add_named_pose",
    std::bind(&BasePlacementServer::handle_add_named_pose, this, _1, _2)
  );

  srv_remove_named_pose_ = this->create_service<base_placement_interfaces::srv::RemoveNamedPose>(
    "remove_named_pose",
    std::bind(&BasePlacementServer::handle_remove_named_pose, this, _1, _2)
  );

  srv_clear_maps_ = this->create_service<base_placement_interfaces::srv::ClearMaps>(
    "clear_maps",
    std::bind(&BasePlacementServer::handle_clear_maps, this, _1, _2)
  );

  srv_get_base_poses_ = this->create_service<base_placement_interfaces::srv::GetBasePoses>(
    "get_base_poses",
    std::bind(&BasePlacementServer::handle_get_base_poses, this, _1, _2)
  );

  // Create publishers
  pub_irm_ = this->create_publisher<reachability_map_visualizer::msg::WorkSpace>("irm", 1);
  pub_base_find_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("base_find_markers", 1);

  RCLCPP_INFO(this->get_logger(), "BasePlacementServer initialized successfully");
  RCLCPP_INFO(this->get_logger(), "  Action: find_base");
  RCLCPP_INFO(this->get_logger(), "  Services: update_reachability_map, get_union_map, update_parameters,");
  RCLCPP_INFO(this->get_logger(), "            add_named_pose, remove_named_pose, clear_maps, get_base_poses");
}

BasePlacementServer::~BasePlacementServer()
{
  RCLCPP_INFO(this->get_logger(), "BasePlacementServer destroyed");
}

void BasePlacementServer::initialize()
{
  // Create the core computation engine
  core_ = std::make_shared<BasePlacementCore>(this->shared_from_this());
  core_->init();

  RCLCPP_INFO(this->get_logger(), "BasePlacementCore initialized");
}

// ============================================================
// ACTION SERVER CALLBACKS
// ============================================================

rclcpp_action::GoalResponse BasePlacementServer::handle_goal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const FindBase::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request with %zu task poses",
    goal->task_poses.size());

  // Validate goal
  if (goal->task_poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Goal rejected: no task poses provided");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->method_index < 0 || goal->method_index > 4) {
    RCLCPP_WARN(this->get_logger(), "Goal rejected: invalid method_index %d", goal->method_index);
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Goal accepted");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BasePlacementServer::handle_cancel(
  const std::shared_ptr<GoalHandleFindBase> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BasePlacementServer::handle_accepted(const std::shared_ptr<GoalHandleFindBase> goal_handle)
{
  // Execute in a separate thread to avoid blocking the executor
  std::thread{std::bind(&BasePlacementServer::execute_find_base, this, _1), goal_handle}.detach();
}

void BasePlacementServer::execute_find_base(const std::shared_ptr<GoalHandleFindBase> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing find_base action");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<FindBase::Feedback>();
  auto result = std::make_shared<FindBase::Result>();

  // Update core parameters from goal
  BasePlacementCore::Method method = static_cast<BasePlacementCore::Method>(goal->method_index);
  core_->setParameters(method, goal->num_base_locations, goal->num_high_score_spheres);

  // If client provided pre-computed high-score spheres, use them
  if (goal->use_provided_spheres && !goal->high_score_sphere_data.empty()) {
    // Reconstruct high_score_sp_ from flattened data
    std::vector<std::vector<double>> high_score_spheres;
    for (size_t i = 0; i + 2 < goal->high_score_sphere_data.size(); i += 3) {
      std::vector<double> sphere = {
        goal->high_score_sphere_data[i],     // x
        goal->high_score_sphere_data[i + 1], // y
        goal->high_score_sphere_data[i + 2]  // z
      };
      high_score_spheres.push_back(sphere);
    }

    RCLCPP_INFO(this->get_logger(),
      "Using %zu pre-computed high-score spheres from client",
      high_score_spheres.size());

    // Set the high-score spheres in the core
    core_->setHighScoreSpheres(high_score_spheres);
  }

  // If client provided pre-computed union map, use it
  if (goal->use_provided_union_map &&
      !goal->base_trns_col_keys.empty() &&
      !goal->base_trns_col_values.empty() &&
      !goal->base_trns_col_counts.empty()) {

    // Reconstruct base_trns_col_ multimap from flattened data
    std::multimap<std::vector<double>, std::vector<double>> base_trns_col;

    size_t key_idx = 0;
    size_t val_idx = 0;

    for (size_t sphere_idx = 0; sphere_idx < goal->base_trns_col_counts.size(); ++sphere_idx) {
      // Extract the key (sphere position - assuming 3 values)
      std::vector<double> key = {
        goal->base_trns_col_keys[key_idx],
        goal->base_trns_col_keys[key_idx + 1],
        goal->base_trns_col_keys[key_idx + 2]
      };
      key_idx += 3;

      // Extract all poses for this key
      int num_poses = goal->base_trns_col_counts[sphere_idx];
      for (int i = 0; i < num_poses; ++i) {
        // Each pose has 7 values: x, y, z, qx, qy, qz, qw
        std::vector<double> pose_vec = {
          goal->base_trns_col_values[val_idx],     // x
          goal->base_trns_col_values[val_idx + 1], // y
          goal->base_trns_col_values[val_idx + 2], // z
          goal->base_trns_col_values[val_idx + 3], // qx
          goal->base_trns_col_values[val_idx + 4], // qy
          goal->base_trns_col_values[val_idx + 5], // qz
          goal->base_trns_col_values[val_idx + 6]  // qw
        };
        val_idx += 7;

        base_trns_col.insert({key, pose_vec});
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "Using pre-computed union map from client: %zu poses across %zu spheres",
      base_trns_col.size(), goal->base_trns_col_counts.size());

    // Set the union map in the core
    core_->setBaseTrnsCol(base_trns_col);
  }

  // Convert named poses to regular poses
  std::vector<geometry_msgs::msg::Pose> task_poses;
  for (const auto& named_pose : goal->task_poses) {
    task_poses.push_back(named_pose.pose);
  }

  // Create feedback callback
  auto feedback_callback = [&](
    const std::string& phase,
    int iteration,
    int total_iterations,
    double progress_percentage,
    const std::string& status_message,
    int candidates_evaluated,
    double current_best_score)
  {
    // Check if goal has been canceled
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    // Update feedback
    feedback->current_phase = phase;
    feedback->iteration = iteration;
    feedback->total_iterations = total_iterations;
    feedback->progress_percentage = progress_percentage;
    feedback->status_message = status_message;
    feedback->candidates_evaluated = candidates_evaluated;
    feedback->current_best_score = current_best_score;

    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_DEBUG(this->get_logger(), "Feedback: %s [%d/%d] %.1f%%",
      phase.c_str(), iteration, total_iterations, progress_percentage);
  };

  // Execute computation
  auto computation_result = core_->findBasePlacements(task_poses, feedback_callback);

  // Check if canceled
  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Goal canceled";
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return;
  }

  // Fill result
  result->success = computation_result.success;
  result->message = computation_result.message;
  result->base_poses = computation_result.base_poses;
  result->scores = computation_result.scores;
  result->best_score = computation_result.best_score;
  result->best_index = computation_result.best_index;
  result->computation_time_seconds = computation_result.computation_time_seconds;
  publish_find_base(result->base_poses);
  if (result->success) {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded with best score: %.4f", result->best_score);
  } else {
    goal_handle->abort(result);
    RCLCPP_WARN(this->get_logger(), "Goal aborted: %s", result->message.c_str());
  }
}
void BasePlacementServer::publish_find_base(const std::vector<geometry_msgs::msg::Pose>& task_poses)
{
  // Create MarkerArray message
  visualization_msgs::msg::MarkerArray markers;

  // Add an arrow for each task pose
  int marker_id = 0;
  for (const auto& pose : task_poses)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "task_poses";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set arrow scale
    marker.scale.x = 0.3;  // Arrow length
    marker.scale.y = 0.03; // Arrow width
    marker.scale.z = 0.03; // Arrow height

    // Set pose
    marker.pose = pose;

    // Set color (green)
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Set lifetime (0 = forever)
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    markers.markers.push_back(marker);
  }

  // Publish the markers
  pub_base_find_->publish(markers);
  RCLCPP_INFO(this->get_logger(), "Published %zu task pose markers", task_poses.size());
} 
// ============================================================
// SERVICE CALLBACKS
// ============================================================

void BasePlacementServer::handle_update_reachability_map(
  const std::shared_ptr<base_placement_interfaces::srv::UpdateReachabilityMap::Request> request,
  std::shared_ptr<base_placement_interfaces::srv::UpdateReachabilityMap::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service call: update_reachability_map");
  RCLCPP_INFO(this->get_logger(), "  IRM file: %s", request->irm_file_path.c_str());

  if (!request->rm_file_path.empty()) {
    RCLCPP_INFO(this->get_logger(), "  RM file: %s", request->rm_file_path.c_str());
  }

  bool success = false;

  if (request->load_irm) {
    // The core will handle all file loading logic
    success = core_->loadReachabilityFromFile(request->irm_file_path, request->rm_file_path);

    if (success) {
      // Get actual data from core for response
      response->resolution = core_->getResolution();
      response->num_spheres_loaded = static_cast<int32_t>(core_->getSphereCollection().size());
      response->message = "Reachability map loaded successfully";
      this->publish_irm();
      RCLCPP_INFO(this->get_logger(),
                  "Successfully loaded reachability data: %d spheres, resolution=%.4f m",
                  response->num_spheres_loaded, response->resolution);
    } else {
      response->message = "Failed to load reachability map from provided file paths";
      response->resolution = 0.0;
      response->num_spheres_loaded = 0;
      RCLCPP_ERROR(this->get_logger(), "Failed to load reachability map");
    }
  } else {
    response->success = false;
    response->message = "load_irm flag not set";
    response->resolution = 0.0;
    response->num_spheres_loaded = 0;
    RCLCPP_WARN(this->get_logger(), "load_irm flag not set in request");
  }

  response->success = success;
}

// Send the IRM map on topic
void BasePlacementServer::publish_irm()
{
  auto ws_msg = std::make_shared<reachability_map_visualizer::msg::WorkSpace>();

  ws_msg->header.stamp = this->get_clock()->now();
  ws_msg->header.frame_id = "base_link";
  ws_msg->resolution = core_->getResolution();

  for (const auto& sphere_pair : core_->getSphereCollection())
  {
    reachability_map_visualizer::msg::WsSphere wss;
    wss.point.x = (sphere_pair.first)[0];
    wss.point.y = (sphere_pair.first)[1];
    wss.point.z = (sphere_pair.first)[2];
    wss.ri = sphere_pair.second;
    ws_msg->ws_spheres.push_back(wss);
  }

  pub_irm_->publish(*ws_msg);
  RCLCPP_INFO(this->get_logger(), "Published IRM with %zu spheres", ws_msg->ws_spheres.size());
}
void BasePlacementServer::handle_get_union_map(
  const std::shared_ptr<base_placement_interfaces::srv::GetUnionMap::Request> request,
  std::shared_ptr<base_placement_interfaces::srv::GetUnionMap::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service call: get_union_map");

  auto workspace = core_->getUnionMap(request->compute_from_current_poses);

  // Convert BasePlacementCore::WorkSpace to base_placement_interfaces::msg::WorkSpace
  response->union_map.header = workspace.header;
  response->union_map.resolution = workspace.resolution;

  for (const auto& ws_sphere : workspace.ws_spheres) {
    base_placement_interfaces::msg::WsSphere msg_sphere;
    msg_sphere.header = ws_sphere.header;
    msg_sphere.center = ws_sphere.point;
    msg_sphere.radius = ws_sphere.ri;
    msg_sphere.poses = ws_sphere.poses;
    response->union_map.ws_spheres.push_back(msg_sphere);
  }

  response->success = true;
  response->message = "Union map computed";

  RCLCPP_INFO(this->get_logger(), "Union map returned with %zu spheres",
    response->union_map.ws_spheres.size());
}

void BasePlacementServer::handle_update_parameters(
  const std::shared_ptr<base_placement_interfaces::srv::UpdateParameters::Request> request,
  std::shared_ptr<base_placement_interfaces::srv::UpdateParameters::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service call: update_parameters");

  // Validate method index
  if (request->method_index < 0 || request->method_index > 4) {
    response->success = false;
    response->message = "Invalid method_index: must be 0-4";
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  BasePlacementCore::Method method = static_cast<BasePlacementCore::Method>(request->method_index);
  core_->setParameters(method, request->num_base_locations, request->num_high_score_spheres);

  response->success = true;
  response->message = "Parameters updated successfully";

  RCLCPP_INFO(this->get_logger(), "Parameters updated: method=%s, base_locations=%d, high_score_spheres=%d",
    BasePlacementCore::getMethodName(method).c_str(),
    request->num_base_locations,
    request->num_high_score_spheres);
}

void BasePlacementServer::handle_add_named_pose(
  const std::shared_ptr<base_placement_interfaces::srv::AddNamedPose::Request> request,
  std::shared_ptr<base_placement_interfaces::srv::AddNamedPose::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service call: add_named_pose '%s'", request->name.c_str());

  bool success = core_->addNamedPose(request->name, request->pose);

  response->success = success;
  response->total_poses = static_cast<int>(core_->getTaskPoses().size());

  if (success) {
    response->message = "Pose added successfully";
  } else {
    response->message = "Failed to add pose";
  }

  RCLCPP_INFO(this->get_logger(), "%s. Total poses: %d",
    response->message.c_str(), response->total_poses);
}

void BasePlacementServer::handle_remove_named_pose(
  const std::shared_ptr<base_placement_interfaces::srv::RemoveNamedPose::Request> request,
  std::shared_ptr<base_placement_interfaces::srv::RemoveNamedPose::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service call: remove_named_pose '%s'", request->name.c_str());

  bool success = core_->removeNamedPose(request->name);

  response->success = success;
  response->total_poses = static_cast<int>(core_->getTaskPoses().size());

  if (success) {
    response->message = "Pose removed successfully";
  } else {
    response->message = "Pose not found";
  }

  RCLCPP_INFO(this->get_logger(), "%s. Total poses: %d",
    response->message.c_str(), response->total_poses);
}

void BasePlacementServer::handle_clear_maps(
  const std::shared_ptr<base_placement_interfaces::srv::ClearMaps::Request> request,
  std::shared_ptr<base_placement_interfaces::srv::ClearMaps::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service call: clear_maps");

  if (request->clear_union_map) {
    core_->clearUnionMap();
    RCLCPP_INFO(this->get_logger(), "  - Union map cleared");
  }

  if (request->clear_reachability_data) {
    core_->clearReachabilityData();
    RCLCPP_INFO(this->get_logger(), "  - Reachability data cleared");
  }

  if (request->clear_task_poses) {
    core_->clearTaskPoses();
    RCLCPP_INFO(this->get_logger(), "  - Task poses cleared");
  }

  if (request->clear_computed_bases) {
    // Clear computed results (handled by clearing computed poses in core)
    RCLCPP_INFO(this->get_logger(), "  - Computed bases cleared");
  }

  response->success = true;
  response->message = "Maps cleared as requested";
}

void BasePlacementServer::handle_get_base_poses(
  const std::shared_ptr<base_placement_interfaces::srv::GetBasePoses::Request> request,
  std::shared_ptr<base_placement_interfaces::srv::GetBasePoses::Response> response)
{
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Service call: get_base_poses");

  response->base_poses = core_->getComputedBasePoses();
  response->scores = core_->getComputedScores();
  response->best_score = core_->getBestScore();

  // Find best index
  response->best_index = -1;
  if (!response->scores.empty()) {
    auto max_it = std::max_element(response->scores.begin(), response->scores.end());
    response->best_index = std::distance(response->scores.begin(), max_it);
  }

  response->success = !response->base_poses.empty();
  if (response->success) {
    response->message = "Base poses retrieved successfully";
  } else {
    response->message = "No computed base poses available";
  }

  RCLCPP_INFO(this->get_logger(), "Returning %zu base poses", response->base_poses.size());
}

// ============================================================
// MAIN FUNCTION
// ============================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BasePlacementServer>();

  // Initialize core after node construction
  node->initialize();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "BasePlacementServer spinning...");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
