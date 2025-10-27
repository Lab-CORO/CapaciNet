#ifndef BASE_PLACEMENT_SERVER_H_
#define BASE_PLACEMENT_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <base_placement_plugin/base_placement_core.h>
#include <base_placement_interfaces/action/find_base.hpp>
#include <base_placement_interfaces/srv/update_reachability_map.hpp>
#include <base_placement_interfaces/srv/get_union_map.hpp>
#include <base_placement_interfaces/srv/update_parameters.hpp>
#include <base_placement_interfaces/srv/add_named_pose.hpp>
#include <base_placement_interfaces/srv/remove_named_pose.hpp>
#include <base_placement_interfaces/srv/clear_maps.hpp>
#include <base_placement_interfaces/srv/get_base_poses.hpp>

#include "reachability_map_visualizer/msg/work_space.hpp"
#include "reachability_map_visualizer/msg/ws_sphere.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <string>
#include <vector>

/*!
 *  \brief     ROS2 Action Server for Base Placement
 *  \details   Provides ROS2 interface (services and action) to BasePlacementCore
 *             Separates computation logic from RViz visualization
 *  \author    Guillaume Dupoiron
 */
class BasePlacementServer : public rclcpp::Node
{
public:
  using FindBase = base_placement_interfaces::action::FindBase;
  using GoalHandleFindBase = rclcpp_action::ServerGoalHandle<FindBase>;

  //! Constructor
  BasePlacementServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  //! Destructor
  virtual ~BasePlacementServer();

  //! Initialize the core (must be called after construction)
  void initialize();

private:
  // ============================================================
  // ACTION SERVER CALLBACKS
  // ============================================================

  //! Handle new goal request
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const FindBase::Goal> goal
  );

  //! Handle goal cancellation request
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFindBase> goal_handle
  );

  //! Handle accepted goal
  void handle_accepted(const std::shared_ptr<GoalHandleFindBase> goal_handle);

  //! Execute the base placement computation (runs in separate thread)
  void execute_find_base(const std::shared_ptr<GoalHandleFindBase> goal_handle);

  // ============================================================
  // SERVICE CALLBACKS
  // ============================================================

  //! Update reachability map from files
  void handle_update_reachability_map(
    const std::shared_ptr<base_placement_interfaces::srv::UpdateReachabilityMap::Request> request,
    std::shared_ptr<base_placement_interfaces::srv::UpdateReachabilityMap::Response> response
  );

  //! Get union map
  void handle_get_union_map(
    const std::shared_ptr<base_placement_interfaces::srv::GetUnionMap::Request> request,
    std::shared_ptr<base_placement_interfaces::srv::GetUnionMap::Response> response
  );

  //! Update parameters (method, num_base_locations, etc.)
  void handle_update_parameters(
    const std::shared_ptr<base_placement_interfaces::srv::UpdateParameters::Request> request,
    std::shared_ptr<base_placement_interfaces::srv::UpdateParameters::Response> response
  );

  //! Add a named pose
  void handle_add_named_pose(
    const std::shared_ptr<base_placement_interfaces::srv::AddNamedPose::Request> request,
    std::shared_ptr<base_placement_interfaces::srv::AddNamedPose::Response> response
  );

  //! Remove a named pose
  void handle_remove_named_pose(
    const std::shared_ptr<base_placement_interfaces::srv::RemoveNamedPose::Request> request,
    std::shared_ptr<base_placement_interfaces::srv::RemoveNamedPose::Response> response
  );

  //! Clear maps (union map, reachability data, etc.)
  void handle_clear_maps(
    const std::shared_ptr<base_placement_interfaces::srv::ClearMaps::Request> request,
    std::shared_ptr<base_placement_interfaces::srv::ClearMaps::Response> response
  );

  //! Get computed base poses
  void handle_get_base_poses(
    const std::shared_ptr<base_placement_interfaces::srv::GetBasePoses::Request> request,
    std::shared_ptr<base_placement_interfaces::srv::GetBasePoses::Response> response
  );

  //! Publish IRM (Inverse Reachability Map) to visualization topic
  void publish_irm();

  //! Publish task poses as arrow markers for visualization
  void publish_find_base(const std::vector<geometry_msgs::msg::Pose>& task_poses);

  // ============================================================
  // MEMBER VARIABLES
  // ============================================================

  //! Core computation engine (no Qt, no RViz)
  std::shared_ptr<BasePlacementCore> core_;

  //! Action server
  rclcpp_action::Server<FindBase>::SharedPtr action_server_;

  //! Service servers
  rclcpp::Service<base_placement_interfaces::srv::UpdateReachabilityMap>::SharedPtr srv_update_reachability_;
  rclcpp::Service<base_placement_interfaces::srv::GetUnionMap>::SharedPtr srv_get_union_map_;
  rclcpp::Service<base_placement_interfaces::srv::UpdateParameters>::SharedPtr srv_update_parameters_;
  rclcpp::Service<base_placement_interfaces::srv::AddNamedPose>::SharedPtr srv_add_named_pose_;
  rclcpp::Service<base_placement_interfaces::srv::RemoveNamedPose>::SharedPtr srv_remove_named_pose_;
  rclcpp::Service<base_placement_interfaces::srv::ClearMaps>::SharedPtr srv_clear_maps_;
  rclcpp::Service<base_placement_interfaces::srv::GetBasePoses>::SharedPtr srv_get_base_poses_;

  //! Publisher for IRM visualization
  rclcpp::Publisher<reachability_map_visualizer::msg::WorkSpace>::SharedPtr pub_irm_;

  //! Publisher for task pose markers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_base_find_;
};

#endif  // BASE_PLACEMENT_SERVER_H_
