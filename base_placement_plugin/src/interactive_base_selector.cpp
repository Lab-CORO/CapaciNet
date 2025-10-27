#include "base_placement_plugin/interactive_base_selector.h"
#include <Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.hpp>

InteractiveBaseSelector::InteractiveBaseSelector(std::shared_ptr<rclcpp::Node> node, QWidget* parent, std::string group_name)
  : widget_(parent), node_(node), count_(0), group_name_(group_name)
{
  init();
}

InteractiveBaseSelector::~InteractiveBaseSelector()
{
  server_.reset();
}

void InteractiveBaseSelector::init()
{
  // Initialize interactive marker server
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "user_intuition", node_);
  
  waypoints_pos_.clear();
  count_ = 0;

  // Initialize colors
  ROBOT_WAY_POINT_COLOR.r = 0.5;
  ROBOT_WAY_POINT_COLOR.g = 0.5;
  ROBOT_WAY_POINT_COLOR.b = 0.5;
  ROBOT_WAY_POINT_COLOR.a = 0.3;

  WAY_POINT_COLOR.r = 0.1;
  WAY_POINT_COLOR.g = 0.5;
  WAY_POINT_COLOR.b = 1.0;
  WAY_POINT_COLOR.a = 1.0;

  WAY_POINT_SCALE_CONTROL.x = 0.28;
  WAY_POINT_SCALE_CONTROL.y = 0.032;
  WAY_POINT_SCALE_CONTROL.z = 0.032;

  INTERACTIVE_MARKER_SCALE = 0.4;

  ARROW_INTER_COLOR.r = 0.2;
  ARROW_INTER_COLOR.g = 0.2;
  ARROW_INTER_COLOR.b = 0.2;
  ARROW_INTER_COLOR.a = 1.0;

  ROBOT_INTER_COLOR.r = 0.0;
  ROBOT_INTER_COLOR.g = 1.0;
  ROBOT_INTER_COLOR.b = 0.0;
  ROBOT_INTER_COLOR.a = 0.5;

  ARROW_INTER_SCALE_CONTROL.x = 0.27;
  ARROW_INTER_SCALE_CONTROL.y = 0.03;
  ARROW_INTER_SCALE_CONTROL.z = 0.03;

  ARROW_INTERACTIVE_SCALE = 0.3;

  // Setup menu handler
  menu_handler_.insert("Delete", 
    [this](const auto& feedback) { this->processFeedback(feedback); });
  
  menu_handler_.setCheckState(
    menu_handler_.insert("Fine adjustment", 
      [this](const auto& feedback) { this->processFeedback(feedback); }),
    interactive_markers::MenuHandler::UNCHECKED);

  // Initialize CreateMarker
  mark_ = std::make_shared<CreateMarker>(node_, group_name_);
  robot_markers_ = mark_->getDefaultMarkers();

  // Initialize transform
  tf2::Vector3 vec(-1, 0, 0);
  tf2::Quaternion quat(0, 0, 0, 1);
  quat.normalize();
  box_pos_.setOrigin(vec);
  box_pos_.setRotation(quat);

  target_frame_ = "base_link";
  RCLCPP_INFO(node_->get_logger(), "The robot model frame is: %s", target_frame_.c_str());
  
  makeInteractiveMarker();
  server_->applyChanges();
  RCLCPP_INFO(node_->get_logger(), "User base placement interactive marker started.");
}

void InteractiveBaseSelector::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  switch (feedback->event_type)
  {
    case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
    {
      tf2::Transform point_pos;
      tf2::fromMsg(feedback->pose, point_pos);
      makeArrow(point_pos, count_);
      break;
    }

    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      tf2::Transform point_pos;
      tf2::fromMsg(feedback->pose, point_pos);
      pointPoseUpdated(point_pos, feedback->marker_name.c_str());
      break;
    }

    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
    {
      // Get the menu item which is pressed
      interactive_markers::MenuHandler::EntryHandle menu_item = feedback->menu_entry_id;
      interactive_markers::MenuHandler::CheckState state;

      menu_handler_.getCheckState(menu_item, state);

      if (menu_item == 1)
      {
        std::string marker_name = feedback->marker_name;
        pointDeleted(marker_name);
        break;
      }
      else
      {
        if (state == interactive_markers::MenuHandler::UNCHECKED)
        {
          RCLCPP_INFO(node_->get_logger(), "The selected marker is shown with 6DOF control");
          menu_handler_.setCheckState(menu_item, interactive_markers::MenuHandler::CHECKED);
          changeMarkerControlAndPose(feedback->marker_name.c_str(), true);
          break;
        }
        else
        {
          menu_handler_.setCheckState(menu_item, interactive_markers::MenuHandler::UNCHECKED);
          RCLCPP_INFO(node_->get_logger(), "The selected marker is shown as default");
          changeMarkerControlAndPose(feedback->marker_name.c_str(), false);
          break;
        }
      }
      break;
    }
  }
  server_->applyChanges();
}

void InteractiveBaseSelector::getWaypoints(std::vector<geometry_msgs::msg::Pose>& waypoints)
{
  waypoints.resize(waypoints_pos_.size());
  for (size_t i = 0; i < waypoints_pos_.size(); i++)
  {
    geometry_msgs::msg::Transform trans =  tf2::toMsg(waypoints_pos_[i]);
    waypoints[i].position.x = trans.translation.x;
    waypoints[i].position.y = trans.translation.y;
    waypoints[i].position.z = trans.translation.z;
    waypoints[i].orientation = trans.rotation;
  }
}

void InteractiveBaseSelector::changeMarkerControlAndPose(std::string marker_name, bool set_control)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  if (!server_->get(marker_name, int_marker))
  {
    RCLCPP_WARN(node_->get_logger(), "Could not get marker: %s", marker_name.c_str());
    return;
  }

  if (set_control)
  {
    int_marker.controls.clear();
    makeArrowControlDetails(int_marker);
  }
  else
  {
    int_marker.controls.clear();
    makeArrowControlDefault(int_marker);
  }

  server_->insert(int_marker);
  menu_handler_.apply(*server_, int_marker.name);
  server_->setCallback(int_marker.name, 
    [this](const auto& feedback) { this->processFeedback(feedback); });
}

visualization_msgs::msg::InteractiveMarkerControl& 
InteractiveBaseSelector::makeArrowControlDefault(visualization_msgs::msg::InteractiveMarker& msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control_menu;
  control_menu.always_visible = true;
  control_menu.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control_menu.name = "menu_select";
  msg.controls.push_back(control_menu);

  visualization_msgs::msg::InteractiveMarkerControl control_move3d;
  control_move3d.always_visible = true;
  control_move3d.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
  control_move3d.name = "move";
  control_move3d.markers.push_back(makeWayPoint(msg));

  visualization_msgs::msg::MarkerArray control_move3d_markers = makeRobotMarker(msg, true);

  for (size_t i = 0; i < control_move3d_markers.markers.size(); ++i)
    control_move3d.markers.push_back(control_move3d_markers.markers[i]);

  msg.controls.push_back(control_move3d);
  return msg.controls.back();
}

visualization_msgs::msg::InteractiveMarkerControl& 
InteractiveBaseSelector::makeArrowControlDetails(visualization_msgs::msg::InteractiveMarker& msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control_menu;
  control_menu.always_visible = true;
  control_menu.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control_menu.name = "menu_select";
  msg.controls.push_back(control_menu);
  control_menu.markers.push_back(makeWayPoint(msg));

  visualization_msgs::msg::MarkerArray control_menu_markers = makeRobotMarker(msg, true);
  for (size_t i = 0; i < control_menu_markers.markers.size(); ++i)
    control_menu.markers.push_back(control_menu_markers.markers[i]);

  visualization_msgs::msg::InteractiveMarkerControl control_view_details;
  control_view_details.always_visible = true;

  // Rotate and move around the x-axis
  control_view_details.orientation.w = 1;
  control_view_details.orientation.x = 1;
  control_view_details.orientation.y = 0;
  control_view_details.orientation.z = 0;
  control_view_details.name = "move_x";
  control_view_details.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_view_details);

  // Rotate and move around the z-axis
  control_view_details.orientation.w = 1;
  control_view_details.orientation.x = 0;
  control_view_details.orientation.y = 1;
  control_view_details.orientation.z = 0;
  control_view_details.name = "rotate_z";
  control_view_details.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_view_details);

  // Rotate and move around the y-axis
  control_view_details.orientation.w = 1;
  control_view_details.orientation.x = 0;
  control_view_details.orientation.y = 0;
  control_view_details.orientation.z = 1;
  control_view_details.name = "move_y";
  control_view_details.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_view_details);
  control_view_details.markers.push_back(makeWayPoint(msg));

  visualization_msgs::msg::MarkerArray control_view_markers = makeRobotMarker(msg, true);
  for (size_t i = 0; i < control_view_markers.markers.size(); ++i)
    control_view_details.markers.push_back(control_view_markers.markers[i]);
  msg.controls.push_back(control_view_details);

  return msg.controls.back();
}

visualization_msgs::msg::Marker 
InteractiveBaseSelector::makeWayPoint(visualization_msgs::msg::InteractiveMarker& /* msg */)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.scale = WAY_POINT_SCALE_CONTROL;
  marker.color = WAY_POINT_COLOR;
  marker.action = visualization_msgs::msg::Marker::ADD;
  return marker;
}

void InteractiveBaseSelector::pointDeleted(std::string marker_name)
{
  for (size_t i = 0; i < waypoints_pos_.size(); i++)
  {
    RCLCPP_DEBUG(node_->get_logger(), 
      "vector before delete: x:%f; y:%f; z:%f",
      waypoints_pos_[i].getOrigin().x(), 
      waypoints_pos_[i].getOrigin().y(),
      waypoints_pos_[i].getOrigin().z());
  }

  // Get the index of the selected marker
  int index = atoi(marker_name.c_str());
  server_->erase(marker_name.c_str());
  waypoints_pos_.erase(waypoints_pos_.begin() + index - 1);

  for (size_t i = 0; i < waypoints_pos_.size(); i++)
  {
    RCLCPP_DEBUG(node_->get_logger(),
      "vector after delete: x:%f; y:%f; z:%f",
      waypoints_pos_[i].getOrigin().x(),
      waypoints_pos_[i].getOrigin().y(),
      waypoints_pos_[i].getOrigin().z());
  }

  // Recreate markers after deletion
  for (int i = index + 1; i <= count_; i++)
  {
    std::stringstream s;
    s << i;
    server_->erase(s.str());
    makeArrow(waypoints_pos_[i - 2], (i - 1));
  }
  count_--;
  server_->applyChanges();
}

void InteractiveBaseSelector::makeArrow(const tf2::Transform& point_pos, int count_arrow)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  RCLCPP_INFO(node_->get_logger(), "Markers frame is: %s", target_frame_.c_str());
  
  int_marker.header.frame_id = target_frame_;
  int_marker.header.stamp = node_->now();
  RCLCPP_DEBUG(node_->get_logger(), "Markers has frame id: %s", int_marker.header.frame_id.c_str());
  
  int_marker.scale = INTERACTIVE_MARKER_SCALE;
  geometry_msgs::msg::Transform trans =  tf2::toMsg(point_pos);
  int_marker.pose.position.x = trans.translation.x;
  int_marker.pose.position.y = trans.translation.y;
  int_marker.pose.position.z = trans.translation.z;
  int_marker.pose.orientation = trans.rotation;
//   int_marker.pose = tf2::toMsg(point_pos);
  
  std::vector<tf2::Transform>::iterator it_pos =
    std::find((waypoints_pos_.begin()), (waypoints_pos_.end() - 1), point_pos);

  // Check if waypoints are empty - first waypoint
  if (waypoints_pos_.empty())
  {
    RCLCPP_INFO(node_->get_logger(), "Adding first arrow!");
    count_arrow++;
    count_ = count_arrow;
    waypoints_pos_.push_back(point_pos);
  }
  // Check if point already exists
  else if ((it_pos == (waypoints_pos_.end())) ||
           (point_pos.getOrigin() != waypoints_pos_.at(count_arrow - 1).getOrigin()))
  {
    count_arrow++;
    count_ = count_arrow;
    waypoints_pos_.push_back(point_pos);
    RCLCPP_INFO(node_->get_logger(), "Adding new arrow!");
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), 
      "There is already an arrow at that location, can't add new one!!");
  }

  std::stringstream s;
  s << count_arrow;
  RCLCPP_DEBUG(node_->get_logger(), 
    "end of make arrow, count is:%d, positions count:%zu", count_, waypoints_pos_.size());
  
  int_marker.name = s.str();
  int_marker.description = s.str();

  makeArrowControlDefault(int_marker);
  server_->insert(int_marker);
  server_->setCallback(int_marker.name,
    [this](const auto& feedback) { this->processFeedback(feedback); });
  menu_handler_.apply(*server_, int_marker.name);
}

void InteractiveBaseSelector::makeInteractiveMarker()
{
  visualization_msgs::msg::InteractiveMarker inter_arrow_marker_;
  inter_arrow_marker_.header.frame_id = target_frame_;
  inter_arrow_marker_.header.stamp = node_->now();
  inter_arrow_marker_.scale = ARROW_INTERACTIVE_SCALE;
  RCLCPP_INFO(node_->get_logger(), "Marker Frame is: %s", target_frame_.c_str());
  
//   inter_arrow_marker_.pose = tf2::toMsg(box_pos_);
  geometry_msgs::msg::Transform trans =  tf2::toMsg(box_pos_);
  inter_arrow_marker_.pose.position.x = trans.translation.x;
  inter_arrow_marker_.pose.position.y = trans.translation.y;
  inter_arrow_marker_.pose.position.z = trans.translation.z;
  inter_arrow_marker_.pose.orientation = trans.rotation;
  inter_arrow_marker_.description = "Base Marker";
  inter_arrow_marker_.name = "add_point_button";
  
  makeInteractiveMarkerControl(inter_arrow_marker_);
  server_->insert(inter_arrow_marker_);
  server_->setCallback(inter_arrow_marker_.name,
    [this](const auto& feedback) { this->processFeedback(feedback); });
}

visualization_msgs::msg::InteractiveMarkerControl& 
InteractiveBaseSelector::makeInteractiveMarkerControl(visualization_msgs::msg::InteractiveMarker& msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control_button;
  control_button.always_visible = true;
  control_button.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control_button.name = "button_interaction";
  control_button.markers.push_back(makeInterArrow(msg));
  
  visualization_msgs::msg::MarkerArray control_button_markers = makeRobotMarker(msg, false);
  for (size_t i = 0; i < control_button_markers.markers.size(); ++i)
    control_button.markers.push_back(control_button_markers.markers[i]);

  msg.controls.push_back(control_button);
  
  visualization_msgs::msg::InteractiveMarkerControl control_inter_arrow;
  control_inter_arrow.always_visible = true;

  // Move around the x-axis
  control_inter_arrow.orientation.w = 1;
  control_inter_arrow.orientation.x = 1;
  control_inter_arrow.orientation.y = 0;
  control_inter_arrow.orientation.z = 0;
  control_inter_arrow.name = "move_x";
  control_inter_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_inter_arrow);

  // Move around the y-axis
  control_inter_arrow.orientation.w = 1;
  control_inter_arrow.orientation.x = 0;
  control_inter_arrow.orientation.y = 0;
  control_inter_arrow.orientation.z = 1;
  control_inter_arrow.name = "move_y";
  control_inter_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_inter_arrow);

  // Rotate around the z-axis
  control_inter_arrow.orientation.w = 1;
  control_inter_arrow.orientation.x = 0;
  control_inter_arrow.orientation.y = 1;
  control_inter_arrow.orientation.z = 0;
  control_inter_arrow.name = "rotate_z";
  control_inter_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_inter_arrow);

  control_inter_arrow.markers.push_back(makeInterArrow(msg));
  return msg.controls.back();
}

visualization_msgs::msg::MarkerArray 
InteractiveBaseSelector::makeRobotMarker(visualization_msgs::msg::InteractiveMarker& msg, bool waypoint)
{
  geometry_msgs::msg::TransformStamped pose;
  pose.transform.translation.x = msg.pose.position.x;
  pose.transform.translation.y = msg.pose.position.y;
  pose.transform.translation.z = msg.pose.position.z;
  pose.transform.rotation = msg.pose.orientation;

  Eigen::Affine3d base_tf = tf2::transformToEigen(pose);
//   Eigen::Affine3d base_tf = tf2::transformToEigen(msg.pose);
  visualization_msgs::msg::MarkerArray markArr;
  visualization_msgs::msg::MarkerArray robot_markers_new = robot_markers_;
  
  for (size_t i = 0; i < robot_markers_.markers.size(); ++i)
  {
    robot_markers_new.markers[i].type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    robot_markers_new.markers[i].mesh_use_embedded_materials = true;
    
    if (waypoint)
      robot_markers_new.markers[i].color = ROBOT_WAY_POINT_COLOR;
    else
      robot_markers_new.markers[i].color = ROBOT_INTER_COLOR;

    robot_markers_new.markers[i].action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::TransformStamped pose;
    pose.transform.translation.x = robot_markers_.markers[i].pose.position.x;
    pose.transform.translation.y = robot_markers_.markers[i].pose.position.y;
    pose.transform.translation.z = robot_markers_.markers[i].pose.position.z;
    pose.transform.rotation = robot_markers_.markers[i].pose.orientation;

    Eigen::Affine3d link_marker = tf2::transformToEigen(pose);
    // Eigen::Affine3d link_marker = tf2::transformToEigen(robot_markers_.markers[i].pose);
    geometry_msgs::msg::Pose new_marker_pose = tf2::toMsg(base_tf * link_marker);
    robot_markers_new.markers[i].pose = new_marker_pose;

    markArr.markers.push_back(robot_markers_new.markers[i]);
  }
  return markArr;
}

visualization_msgs::msg::Marker 
InteractiveBaseSelector::makeInterArrow(visualization_msgs::msg::InteractiveMarker& /* msg */)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.scale = ARROW_INTER_SCALE_CONTROL;
  marker.color = ARROW_INTER_COLOR;
  return marker;
}

void InteractiveBaseSelector::parseWayPoints()
{
  geometry_msgs::msg::Pose target_pose;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  for (size_t i = 0; i < waypoints_pos_.size(); i++)
  {
    geometry_msgs::msg::Transform trans =  tf2::toMsg(waypoints_pos_[i]);
    target_pose.position.x = trans.translation.x;
    target_pose.position.y = trans.translation.y;
    target_pose.position.z = trans.translation.z;
    target_pose.orientation = trans.rotation;
    // target_pose = tf2::toMsg(waypoints_pos_[i]);
    
    waypoints.push_back(target_pose);
  }
  Q_EMIT baseWayPoints_signal(waypoints);
}

void InteractiveBaseSelector::clearAllPointsRviz()
{
  waypoints_pos_.clear();
  server_->clear();
  count_ = 0;
  makeInteractiveMarker();
  server_->applyChanges();
}

void InteractiveBaseSelector::getRobotModelFrame_slot(const tf2::Transform end_effector)
{
  target_frame_ = "base_link";
  RCLCPP_INFO(node_->get_logger(), "The robot model frame is: %s", target_frame_.c_str());
  box_pos_ = end_effector;
  clearAllPointsRviz();
  count_ = 0;
  makeInteractiveMarker();
  server_->applyChanges();
}

void InteractiveBaseSelector::pointPoseUpdated(const tf2::Transform& point_pos, const char* marker_name)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Transform trans =  tf2::toMsg(point_pos);
  pose.position.x = trans.translation.x;
  pose.position.y = trans.translation.y;
  pose.position.z = trans.translation.z;
  pose.orientation = trans.rotation;
  std::stringstream s;
  
  if (strcmp("add_point_button", marker_name) == 0)
  {
    box_pos_ = point_pos;
    s << "add_point_button";
  }
  else
  {
    int index = atoi(marker_name);
    if (index > static_cast<int>(waypoints_pos_.size()))
    {
      return;
    }
    waypoints_pos_[index - 1] = point_pos;
    s << index;
  }
  server_->setPose(s.str(), pose);
  server_->applyChanges();
}