/*
 * Test Program for Base Placement Pipeline
 *
 * This program tests the complete base placement pipeline using ROS2 services and actions:
 * 1. Declares ROS parameters for IRM and RM file paths
 * 2. Calls update_reachability_map service with the file paths
 * 3. Calls add_named_pose service to create a test pose
 * 4. Calls find_base action to compute optimal base placement
 * 5. Calls get_base_poses service to retrieve results
 *
 * Usage:
 *   ros2 run base_placement_plugin test_pipeline \
 *     --ros-args \
 *     -p irm_path:=/path/to/IRM_0.1.h5 \
 *     -p rm_path:=/path/to/master_ik_data0.1.h5
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <base_placement_interfaces/srv/update_reachability_map.hpp>
#include <base_placement_interfaces/srv/add_named_pose.hpp>
#include <base_placement_interfaces/srv/get_base_poses.hpp>
#include <base_placement_interfaces/action/find_base.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>

using namespace std::chrono_literals;

class TestPipeline : public rclcpp::Node
{
public:
  using FindBase = base_placement_interfaces::action::FindBase;
  using GoalHandleFindBase = rclcpp_action::ClientGoalHandle<FindBase>;

  TestPipeline() : Node("test_pipeline")
  {
    // Declare parameters with default values
    this->declare_parameter<std::string>("irm_path",
      "/home/ros2_ws/src/CapaciNet/data_generation/data/IRM_0.1.h5");
    this->declare_parameter<std::string>("rm_path",
      "/home/ros2_ws/src/CapaciNet/data_generation/data/master_ik_data0.1.h5");

    // Get parameters
    irm_path_ = this->get_parameter("irm_path").as_string();
    rm_path_ = this->get_parameter("rm_path").as_string();

    // Create service clients
    update_reachability_client_ = this->create_client<base_placement_interfaces::srv::UpdateReachabilityMap>(
      "update_reachability_map");
    add_named_pose_client_ = this->create_client<base_placement_interfaces::srv::AddNamedPose>(
      "add_named_pose");
    get_base_poses_client_ = this->create_client<base_placement_interfaces::srv::GetBasePoses>(
      "get_base_poses");

    // Create action client
    find_base_action_client_ = rclcpp_action::create_client<FindBase>(this, "find_base");

    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "  Base Placement Pipeline Test");
    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "IRM path: %s", irm_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "RM path:  %s", rm_path_.c_str());
  }

  bool run_test()
  {
    // Step 1: Update reachability map
    if (!step1_update_reachability_map()) {
      return false;
    }

    // Step 2: Add named pose
    if (!step2_add_named_pose()) {
      return false;
    }

    // Step 3: Find base using action
    if (!step3_find_base()) {
      return false;
    }

    // Step 4: Get base poses
    if (!step4_get_base_poses()) {
      return false;
    }

    return true;
  }

private:
  bool step1_update_reachability_map()
  {
    RCLCPP_INFO(this->get_logger(), "\n--- Step 1: Update Reachability Map ---");

    // Wait for service
    if (!update_reachability_client_->wait_for_service(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Service 'update_reachability_map' not available");
      return false;
    }

    // Create request
    auto request = std::make_shared<base_placement_interfaces::srv::UpdateReachabilityMap::Request>();
    request->irm_file_path = irm_path_;
    request->rm_file_path = rm_path_;
    request->load_irm = true;
    request->load_rm = true;

    RCLCPP_INFO(this->get_logger(), "Calling update_reachability_map service...");

    // Call service synchronously
    auto future = update_reachability_client_->async_send_request(request);

    // Wait for response
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 30s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out");
      return false;
    }

    auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(this->get_logger(), "Service returned failure: %s", response->message.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "✓ Reachability map updated successfully");
    RCLCPP_INFO(this->get_logger(), "  Spheres loaded: %d", response->num_spheres_loaded);
    RCLCPP_INFO(this->get_logger(), "  Resolution: %.3f m", response->resolution);
    RCLCPP_INFO(this->get_logger(), "  Message: %s", response->message.c_str());

    return true;
  }

  bool step2_add_named_pose()
  {
    RCLCPP_INFO(this->get_logger(), "\n--- Step 2: Add Named Pose ---");

    // Wait for service
    if (!add_named_pose_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Service 'add_named_pose' not available");
      return false;
    }

    // Create request with test pose at x=2, y=0, z=0.5
    auto request = std::make_shared<base_placement_interfaces::srv::AddNamedPose::Request>();
    request->name = "test_pose_1";
    request->pose.position.x = 2.0;
    request->pose.position.y = 0.0;
    request->pose.position.z = 0.5;
    request->pose.orientation.w = 1.0;
    request->pose.orientation.x = 0.0;
    request->pose.orientation.y = 0.0;
    request->pose.orientation.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Adding pose '%s' at [%.1f, %.1f, %.1f]...",
                request->name.c_str(),
                request->pose.position.x,
                request->pose.position.y,
                request->pose.position.z);

    // Call service synchronously
    auto future = add_named_pose_client_->async_send_request(request);

    // Wait for response
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out");
      return false;
    }

    auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(this->get_logger(), "Service returned failure: %s", response->message.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "✓ Pose added successfully");
    RCLCPP_INFO(this->get_logger(), "  Total poses: %d", response->total_poses);
    RCLCPP_INFO(this->get_logger(), "  Message: %s", response->message.c_str());

    // Store pose name for later use
    test_pose_name_ = request->name;

    return true;
  }

  bool step3_find_base()
  {
    RCLCPP_INFO(this->get_logger(), "\n--- Step 3: Find Base Placement ---");

    // Wait for action server
    if (!find_base_action_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server 'find_base' not available");
      return false;
    }

    // Create goal
    auto goal_msg = FindBase::Goal();

    // Add the test pose to task_poses
    base_placement_interfaces::msg::PoseNamed pose_named;
    pose_named.name = test_pose_name_;
    pose_named.pose.position.x = 2.0;
    pose_named.pose.position.y = 0.0;
    pose_named.pose.position.z = 0.5;
    pose_named.pose.orientation.w = 1.0;
    pose_named.pose.orientation.x = 0.0;
    pose_named.pose.orientation.y = 0.0;
    pose_named.pose.orientation.z = 0.0;
    goal_msg.task_poses.push_back(pose_named);

    // Set parameters
    goal_msg.method_index = 0;  // PCA method
    goal_msg.num_base_locations = 1;
    goal_msg.num_high_score_spheres = 1;
    goal_msg.use_provided_spheres = false;
    goal_msg.use_provided_union_map = false;

    RCLCPP_INFO(this->get_logger(), "Sending goal to find_base action...");
    RCLCPP_INFO(this->get_logger(), "  Method: PCA (index 0)");
    RCLCPP_INFO(this->get_logger(), "  Num base locations: %d", goal_msg.num_base_locations);
    RCLCPP_INFO(this->get_logger(), "  Num high score spheres: %d", goal_msg.num_high_score_spheres);
    RCLCPP_INFO(this->get_logger(), "  Task poses: 1 (%s)", test_pose_name_.c_str());

    // Send goal with callbacks
    auto send_goal_options = rclcpp_action::Client<FindBase>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandleFindBase::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
          goal_accepted_ = false;
        } else {
          RCLCPP_INFO(this->get_logger(), "✓ Goal accepted by server, waiting for result...");
          goal_accepted_ = true;
        }
      };

    send_goal_options.feedback_callback =
      [this](GoalHandleFindBase::SharedPtr,
             const std::shared_ptr<const FindBase::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "  [%3.0f%%] %s: %s",
                    feedback->progress_percentage,
                    feedback->current_phase.c_str(),
                    feedback->status_message.c_str());
      };

    send_goal_options.result_callback =
      [this](const GoalHandleFindBase::WrappedResult& result) {
        goal_done_ = true;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            goal_success_ = result.result->success;
            if (goal_success_) {
              RCLCPP_INFO(this->get_logger(), "✓ Action completed successfully!");
              RCLCPP_INFO(this->get_logger(), "  Best score: %.2f", result.result->best_score);
              RCLCPP_INFO(this->get_logger(), "  Computation time: %.3f s",
                          result.result->computation_time_seconds);
              RCLCPP_INFO(this->get_logger(), "  Base poses found: %zu",
                          result.result->base_poses.size());
            } else {
              RCLCPP_ERROR(this->get_logger(), "Action succeeded but computation failed: %s",
                           result.result->message.c_str());
            }
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Action was aborted");
            goal_success_ = false;
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Action was canceled");
            goal_success_ = false;
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            goal_success_ = false;
            break;
        }
      };

    // Reset flags
    goal_accepted_ = false;
    goal_done_ = false;
    goal_success_ = false;

    // Send goal
    auto goal_handle_future = find_base_action_client_->async_send_goal(goal_msg, send_goal_options);

    // Wait for goal to be accepted
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future, 5s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
      return false;
    }

    // Wait for result
    RCLCPP_INFO(this->get_logger(), "Waiting for action to complete...");
    auto start_time = std::chrono::steady_clock::now();
    while (!goal_done_ && rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(100ms);

      // Timeout after 60 seconds
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > 60s) {
        RCLCPP_ERROR(this->get_logger(), "Action timed out after 60 seconds");
        return false;
      }
    }

    return goal_success_;
  }

  bool step4_get_base_poses()
  {
    RCLCPP_INFO(this->get_logger(), "\n--- Step 4: Get Base Poses ---");

    // Wait for service
    if (!get_base_poses_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Service 'get_base_poses' not available");
      return false;
    }

    // Create request (empty)
    auto request = std::make_shared<base_placement_interfaces::srv::GetBasePoses::Request>();

    RCLCPP_INFO(this->get_logger(), "Calling get_base_poses service...");

    // Call service synchronously
    auto future = get_base_poses_client_->async_send_request(request);

    // Wait for response
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out");
      return false;
    }

    auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(this->get_logger(), "Service returned failure: %s", response->message.c_str());
      return false;
    }

    // Print results
    RCLCPP_INFO(this->get_logger(), "✓ Successfully retrieved base poses");
    RCLCPP_INFO(this->get_logger(), "\n===========================================");
    RCLCPP_INFO(this->get_logger(), "  RESULTS");
    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "Number of base poses: %zu", response->base_poses.size());
    RCLCPP_INFO(this->get_logger(), "Best score: %.2f", response->best_score);
    RCLCPP_INFO(this->get_logger(), "Best index: %d", response->best_index);

    if (!response->base_poses.empty()) {
      RCLCPP_INFO(this->get_logger(), "\nBase Poses:");
      for (size_t i = 0; i < response->base_poses.size(); ++i) {
        const auto& pose = response->base_poses[i];
        double score = i < response->scores.size() ? response->scores[i] : 0.0;

        RCLCPP_INFO(this->get_logger(), "  Pose %zu (score: %.2f):", i, score);
        RCLCPP_INFO(this->get_logger(), "    Position: [%.3f, %.3f, %.3f]",
                    pose.position.x, pose.position.y, pose.position.z);
        RCLCPP_INFO(this->get_logger(), "    Orientation: [%.3f, %.3f, %.3f, %.3f]",
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w);

        if (static_cast<int>(i) == response->best_index) {
          RCLCPP_INFO(this->get_logger(), "    ⭐ BEST POSE");
        }
      }
    }

    RCLCPP_INFO(this->get_logger(), "===========================================\n");

    return true;
  }

  // Member variables
  std::string irm_path_;
  std::string rm_path_;
  std::string test_pose_name_;

  rclcpp::Client<base_placement_interfaces::srv::UpdateReachabilityMap>::SharedPtr update_reachability_client_;
  rclcpp::Client<base_placement_interfaces::srv::AddNamedPose>::SharedPtr add_named_pose_client_;
  rclcpp::Client<base_placement_interfaces::srv::GetBasePoses>::SharedPtr get_base_poses_client_;
  rclcpp_action::Client<FindBase>::SharedPtr find_base_action_client_;

  bool goal_accepted_ = false;
  bool goal_done_ = false;
  bool goal_success_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto test_node = std::make_shared<TestPipeline>();

  // Run the test
  bool success = test_node->run_test();

  if (success) {
    RCLCPP_INFO(test_node->get_logger(), "\n✓ ALL TESTS PASSED!");
    rclcpp::shutdown();
    return 0;
  } else {
    RCLCPP_ERROR(test_node->get_logger(), "\n✗ TEST FAILED");
    rclcpp::shutdown();
    return 1;
  }
}
