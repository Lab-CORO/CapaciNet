// add_way_point_ros2.cpp
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <functional>  // Pour std::bind

using namespace std::placeholders;  // Pour _1, _2, etc.

#include <QHBoxLayout>
#include <QFileDialog>
#include <QMessageBox>

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yaml-cpp/yaml.h>

// includes de ton UI et des classes PlaceBase/widgets déjà existantes
#include "base_placement_plugin/base_placement_panel.h" // adapte si le header est renommé

namespace base_placement_plugin
{

BasePlacementPanel::BasePlacementPanel(QWidget* parent)
: rviz_common::Panel(parent),
  count_(0),
  node_(nullptr)
{
  setObjectName("BasePlacementPlannerPlugin");

  // Le node sera initialisé dans onInitialize() en utilisant le node RViz
  // Cela garantit que le node est correctement spinné

  // couleurs / échelles
  WAY_POINT_COLOR.r = 1.0;
  WAY_POINT_COLOR.g = 0.20;
  WAY_POINT_COLOR.b = 1.0;
  WAY_POINT_COLOR.a = 1.0;

  WAY_POINT_SCALE_CONTROL.x = 0.28;
  WAY_POINT_SCALE_CONTROL.y = 0.032;
  WAY_POINT_SCALE_CONTROL.z = 0.032;

  INTERACTIVE_MARKER_SCALE = 0.4;

  ARROW_INTER_COLOR.r = 0.8;
  ARROW_INTER_COLOR.g = 0.2;
  ARROW_INTER_COLOR.b = 0.1;
  ARROW_INTER_COLOR.a = 1.0;

  ARROW_INTER_SCALE_CONTROL.x = 0.27;
  ARROW_INTER_SCALE_CONTROL.y = 0.03;
  ARROW_INTER_SCALE_CONTROL.z = 0.03;

  ARROW_INTERACTIVE_SCALE = 0.3;

  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "Constructor created;");
}

BasePlacementPanel::~BasePlacementPanel()
{
  server_.reset();
}

void BasePlacementPanel::onInitialize()
{
  // Obtenir le node RViz au lieu de créer un nouveau node
  // Le node RViz est déjà spinné correctement par RViz
  auto rviz_ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (rviz_ros_node)
  {
    node_ = rviz_ros_node->get_raw_node();
    RCLCPP_INFO(node_->get_logger(), "Using RViz node for interactive markers");
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("base_placement_plugin"), "Failed to get RViz node!");
    return;
  }

  // Recréer le serveur avec le node RViz
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("marker_server", node_);

  // Initialize action client instead of PlaceBase
  action_client_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  find_base_action_client_ = rclcpp_action::create_client<base_placement_interfaces::action::FindBase>(
    node_, "find_base", action_client_cb_group_);

  // Initialize service clients for reachability data and other operations
  update_reachability_client_ = node_->create_client<base_placement_interfaces::srv::UpdateReachabilityMap>(
    "update_reachability_map", rmw_qos_profile_services_default, action_client_cb_group_);
  update_parameters_client_ = node_->create_client<base_placement_interfaces::srv::UpdateParameters>(
    "update_parameters", rmw_qos_profile_services_default, action_client_cb_group_);

  widget_ = new widgets::BasePlacementWidget("~");
  // Passer le nœud ROS2 au widget pour qu'il puisse créer AddRobotBase
  static_cast<widgets::BasePlacementWidget*>(widget_)->setNode(node_);
  this->parentWidget()->resize(widget_->width(), widget_->height());
  QHBoxLayout* main_layout = new QHBoxLayout(this);
  main_layout->addWidget(widget_);

  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "initializing..");

  menu_handler_.insert("Delete", std::bind(&BasePlacementPanel::processFeedback, this, std::placeholders::_1));
  {
    auto h = menu_handler_.insert("Fine adjustment", std::bind(&BasePlacementPanel::processFeedback, this, std::placeholders::_1));
    menu_handler_.setCheckState(h, interactive_markers::MenuHandler::UNCHECKED);
  }

  // Initialize method selection and parameters
  selected_method_ = 0;
  num_base_locations_ = 5;
  num_high_score_spheres_ = 10;

  // Send initial data to widget
  // Widget -> this connections
  connect(widget_, SIGNAL(basePlacementParamsFromUI_signal(int, int)), this, SLOT(setBasePlaceParams(int, int)));

  // Signal for file paths (server will load the files)
  connect(widget_, SIGNAL(reachabilityFilePaths_signal(QString, QString)),
          this, SLOT(loadReachabilityFromFiles(QString, QString)));

  connect(widget_, SIGNAL(SendSelectedMethod(int)), this, SLOT(setSelectedMethod(int)));
  connect(widget_, SIGNAL(SendSelectedOpType(int)), this, SLOT(setSelectedOpType(int)));
  connect(widget_, SIGNAL(SendBasePoses(std::vector<geometry_msgs::msg::Pose>)), this, SLOT(setUserBasePoses(std::vector<geometry_msgs::msg::Pose>)));

  connect(widget_, SIGNAL(addPoint(tf2::Transform)), this, SLOT(addPointFromUI(tf2::Transform)));
  connect(widget_, SIGNAL(pointDelUI_signal(std::string)), this, SLOT(pointDeleted(std::string)));
  connect(widget_, SIGNAL(parseWayPointBtn_signal()), this, SLOT(parseWayPoints()));
  connect(widget_, SIGNAL(pointPosUpdated_signal(const tf2::Transform&, const char*)), this,
          SLOT(pointPoseUpdated(const tf2::Transform&, const char*)));
  connect(widget_, SIGNAL(saveToFileBtn_press()), this, SLOT(saveWayPointsToFile()));
  connect(widget_, SIGNAL(clearAllPoints_signal()), this, SLOT(clearAllPointsRViz()));

  // this -> widget connections
  connect(this, SIGNAL(getinitialmarkerFrame_signal(const tf2::Transform)), widget_,
          SLOT(setAddPointUIStartPos(const tf2::Transform)));
  connect(this, SIGNAL(basePlacementProcessStarted()), widget_, SLOT(PlaceBaseStartedHandler()));
  connect(this, SIGNAL(basePlacementProcessFinished()), widget_, SLOT(PlaceBaseFinishedHandler()));
  connect(this, SIGNAL(basePlacementProcessCompleted(double)), widget_, SLOT(PlaceBaseCompleted_slot(double)));
  connect(this, SIGNAL(sendBasePlaceMethods_signal(std::vector< std::string >)), widget_,
          SLOT(getBasePlacePlanMethod(std::vector< std::string >)));
  connect(this, SIGNAL(sendOuputType_signal(std::vector< std::string >)), widget_,
          SLOT(getOutputType(std::vector< std::string >)));

  connect(this, SIGNAL(addPointRViz(const tf2::Transform&, const int)), widget_,
          SLOT(insertRow(const tf2::Transform&, const int)));
  connect(this, SIGNAL(pointPoseUpdatedRViz(const tf2::Transform&, const char*)), widget_,
          SLOT(pointPosUpdated_slot(const tf2::Transform&, const char*)));
  connect(this, SIGNAL(pointDeleteRviz(int)), widget_, SLOT(removeRow(int)));

  // ========== EMIT SIGNALS AFTER CONNECTIONS ARE ESTABLISHED ==========
  // Send initial transform
  tf2::Transform identity_transform;
  identity_transform.setIdentity();
  Q_EMIT getinitialmarkerFrame_signal(identity_transform);

  // Send method names to widget
  std::vector<std::string> method_names = {
    "PrincipalComponentAnalysis",
    "GraspReachabilityScore",
    "IKSolutionScore",
    "VerticalRobotModel",
    "UserIntuition"
  };
  Q_EMIT sendBasePlaceMethods_signal(method_names);
  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "Sent %zu method names to widget", method_names.size());

  // Send output type names to widget
  std::vector<std::string> output_types = {"Arrows", "Manipulator", "RobotModel"};
  Q_EMIT sendOuputType_signal(output_types);
  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "Sent %zu output types to widget", output_types.size());

  // Initialiser le frame_id et créer le marqueur interactif initial
  target_frame_ = "base_link";  // Frame par défaut
  box_pos_.setIdentity();       // Position initiale à l'origine
  makeInteractiveMarker();      // Créer le marqueur "add_point_button"
  server_->applyChanges();      // Publier les changements vers RViz

  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "ready.");
}

void BasePlacementPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  QString text_entry;
  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "rviz: Initializing the user interaction planning panel");
  if (config.mapGetString("TextEntry", &text_entry))
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("base_placement_plugin"), "Loaded TextEntry with value: " << text_entry.toStdString());
  }
  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "rviz Initialization Finished reading config file");
}

void BasePlacementPanel::save(rviz_common::Config config) const
{
  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "Saving configuration");
  rviz_common::Panel::save(config);
  config.mapSetValue("TextEntry", QString::fromStdString(std::string("test_field")));
}

void BasePlacementPanel::addPointFromUI(const tf2::Transform point_pos)
{
  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "Point Added");
  makeArrow(point_pos, count_);
  server_->applyChanges();
}

void BasePlacementPanel::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  // event_type constants are the same names in ROS2 messages
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
      Q_EMIT pointPoseUpdatedRViz(point_pos, feedback->marker_name.c_str());
      break;
    }
    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
    {
      auto menu_item = feedback->menu_entry_id;
      interactive_markers::MenuHandler::CheckState state;
      menu_handler_.getCheckState(menu_item, state);

      if (menu_item == 1)
      {
        std::string marker_name = feedback->marker_name;
        int marker_nr = atoi(marker_name.c_str());
        Q_EMIT pointDeleteRviz(marker_nr);
        pointDeleted(marker_name);
        break;
      }
      else
      {
        if (state == interactive_markers::MenuHandler::UNCHECKED)
        {
          RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "The selected marker is shown with 6DOF control");
          menu_handler_.setCheckState(menu_item, interactive_markers::MenuHandler::CHECKED);
          changeMarkerControlAndPose(feedback->marker_name.c_str(), true);
          break;
        }
        else
        {
          menu_handler_.setCheckState(menu_item, interactive_markers::MenuHandler::UNCHECKED);
          RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "The selected marker is shown as default");
          changeMarkerControlAndPose(feedback->marker_name.c_str(), false);
          break;
        }
      }
      break;
    }
  }
  server_->applyChanges();
}

void BasePlacementPanel::pointPoseUpdated(const tf2::Transform& point_pos, const char* marker_name)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = point_pos.getOrigin().x();
  pose.position.y = point_pos.getOrigin().y();
  pose.position.z = point_pos.getOrigin().z();
  pose.orientation.x = point_pos.getRotation().x();
  pose.orientation.y = point_pos.getRotation().y();
  pose.orientation.z = point_pos.getRotation().z();
  pose.orientation.w = point_pos.getRotation().w();
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

visualization_msgs::msg::Marker BasePlacementPanel::makeWayPoint(visualization_msgs::msg::InteractiveMarker& /* msg */)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.scale = WAY_POINT_SCALE_CONTROL;
  marker.color = WAY_POINT_COLOR;
  // ROS2 msg action field not present on marker (was in visualization_msgs/Marker in ROS1)
  // marker.action = visualization_msgs::Marker::ADD; // not necessary
  return marker;
}

visualization_msgs::msg::InteractiveMarkerControl& BasePlacementPanel::makeArrowControlDefault(visualization_msgs::msg::InteractiveMarker& msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control_menu;
  control_menu.always_visible = true;
  control_menu.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control_menu.name = "menu_select";
  msg.controls.push_back(control_menu);
  msg.controls.back().markers.push_back(makeWayPoint(msg));

  visualization_msgs::msg::InteractiveMarkerControl control_move3d;
  control_move3d.always_visible = true;
  control_move3d.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
  control_move3d.name = "move";
  control_move3d.markers.push_back(makeWayPoint(msg));
  msg.controls.push_back(control_move3d);

  return msg.controls.back();
}

visualization_msgs::msg::InteractiveMarkerControl& BasePlacementPanel::makeArrowControlDetails(visualization_msgs::msg::InteractiveMarker& msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control_menu;
  control_menu.always_visible = true;
  control_menu.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control_menu.name = "menu_select";
  msg.controls.push_back(control_menu);
  msg.controls.back().markers.push_back(makeWayPoint(msg));

  visualization_msgs::msg::InteractiveMarkerControl control_view_details;
  control_view_details.always_visible = true;

  // x-axis
  control_view_details.orientation.w = 1.0;
  control_view_details.orientation.x = 1.0;
  control_view_details.orientation.y = 0.0;
  control_view_details.orientation.z = 0.0;
  control_view_details.name = "rotate_x";
  control_view_details.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_view_details);
  control_view_details.name = "move_x";
  control_view_details.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_view_details);

  // z-axis
  control_view_details.orientation.w = 1.0;
  control_view_details.orientation.x = 0.0;
  control_view_details.orientation.y = 1.0;
  control_view_details.orientation.z = 0.0;
  control_view_details.name = "rotate_z";
  control_view_details.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_view_details);
  control_view_details.name = "move_z";
  control_view_details.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_view_details);

  // y-axis
  control_view_details.orientation.w = 1.0;
  control_view_details.orientation.x = 0.0;
  control_view_details.orientation.y = 0.0;
  control_view_details.orientation.z = 1.0;
  control_view_details.name = "rotate_y";
  control_view_details.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_view_details);
  control_view_details.name = "move_y";
  control_view_details.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_view_details);

  msg.controls.back().markers.push_back(makeWayPoint(msg));
  return msg.controls.back();
}

void BasePlacementPanel::makeArrow(const tf2::Transform& point_pos, int count_arrow)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("base_placement_plugin"), "Markers frame is: " << target_frame_);
  int_marker.header.frame_id = target_frame_;
  int_marker.scale = INTERACTIVE_MARKER_SCALE;
  // Convert tf2::Transform to geometry_msgs::msg::Pose
  int_marker.pose.position.x = point_pos.getOrigin().x();
  int_marker.pose.position.y = point_pos.getOrigin().y();
  int_marker.pose.position.z = point_pos.getOrigin().z();
  int_marker.pose.orientation.x = point_pos.getRotation().x();
  int_marker.pose.orientation.y = point_pos.getRotation().y();
  int_marker.pose.orientation.z = point_pos.getRotation().z();
  int_marker.pose.orientation.w = point_pos.getRotation().w();

  // find if duplicate
  auto it_pos = std::find(waypoints_pos_.begin(), waypoints_pos_.end(), point_pos);

  if (waypoints_pos_.empty())
  {
    RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "Adding first arrow!");
    count_arrow++;
    count_ = count_arrow;
    waypoints_pos_.push_back(point_pos);
    Q_EMIT addPointRViz(point_pos, count_);
  }
  else if ((it_pos == waypoints_pos_.end()) ||
           (point_pos.getOrigin() != waypoints_pos_.at(count_arrow - 1).getOrigin()))
  {
    count_arrow++;
    count_ = count_arrow;
    waypoints_pos_.push_back(point_pos);
    RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "Adding new arrow!");
    Q_EMIT addPointRViz(point_pos, count_);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "There is already a arrow at that location, can't add new one!!");
  }

  std::stringstream s;
  s << count_arrow;
  int_marker.name = s.str();
  int_marker.description = s.str();

  makeArrowControlDefault(int_marker);
  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&BasePlacementPanel::processFeedback, this, std::placeholders::_1));
  menu_handler_.apply(*server_, int_marker.name);
}

void BasePlacementPanel::changeMarkerControlAndPose(std::string marker_name, bool set_control)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  server_->get(marker_name, int_marker);

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
  server_->setCallback(int_marker.name, std::bind(&BasePlacementPanel::processFeedback, this, std::placeholders::_1));
}

void BasePlacementPanel::pointDeleted(std::string marker_name)
{
  for (size_t i = 0; i < waypoints_pos_.size(); ++i)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("base_placement_plugin"),
                        "vector before delete: x:" << waypoints_pos_[i].getOrigin().x()
                                                   << "; y:" << waypoints_pos_[i].getOrigin().y()
                                                   << "; z:" << waypoints_pos_[i].getOrigin().z());
  }

  int index = atoi(marker_name.c_str());
  server_->erase(marker_name);
  waypoints_pos_.erase(waypoints_pos_.begin() + index - 1);

  for (size_t i = 0; i < waypoints_pos_.size(); ++i)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("base_placement_plugin"),
                        "vector after delete: x:" << waypoints_pos_[i].getOrigin().x()
                                                  << "; y:" << waypoints_pos_[i].getOrigin().y()
                                                  << "; z:" << waypoints_pos_[i].getOrigin().z());
  }

  for (int i = index + 1; i <= count_; ++i)
  {
    std::stringstream s;
    s << i;
    server_->erase(s.str());
    makeArrow(waypoints_pos_[i - 2], (i - 1));
  }
  count_--;
  server_->applyChanges();
}

visualization_msgs::msg::Marker BasePlacementPanel::makeInterArrow(visualization_msgs::msg::InteractiveMarker& /* msg */)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.scale = ARROW_INTER_SCALE_CONTROL;
  marker.color = ARROW_INTER_COLOR;
  return marker;
}

visualization_msgs::msg::InteractiveMarkerControl& BasePlacementPanel::makeInteractiveMarkerControl(visualization_msgs::msg::InteractiveMarker& msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control_button;
  control_button.always_visible = true;
  control_button.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control_button.name = "button_interaction";
  control_button.markers.push_back(makeInterArrow(msg));
  msg.controls.push_back(control_button);

  visualization_msgs::msg::InteractiveMarkerControl control_inter_arrow;
  // x
  control_inter_arrow.orientation.w = 1; control_inter_arrow.orientation.x = 1; control_inter_arrow.orientation.y = 0; control_inter_arrow.orientation.z = 0;
  control_inter_arrow.name = "rotate_x";
  control_inter_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_inter_arrow);
  control_inter_arrow.name = "move_x";
  control_inter_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_inter_arrow);

  // z
  control_inter_arrow.orientation.w = 1; control_inter_arrow.orientation.x = 0; control_inter_arrow.orientation.y = 1; control_inter_arrow.orientation.z = 0;
  control_inter_arrow.name = "rotate_z";
  control_inter_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_inter_arrow);
  control_inter_arrow.name = "move_z";
  control_inter_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_inter_arrow);

  // y
  control_inter_arrow.orientation.w = 1; control_inter_arrow.orientation.x = 0; control_inter_arrow.orientation.y = 0; control_inter_arrow.orientation.z = 1;
  control_inter_arrow.name = "rotate_y";
  control_inter_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_inter_arrow);
  control_inter_arrow.name = "move_y";
  control_inter_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_inter_arrow);

  control_inter_arrow.markers.push_back(makeInterArrow(msg));
  return msg.controls.back();
}

void BasePlacementPanel::makeInteractiveMarker()
{
  visualization_msgs::msg::InteractiveMarker inter_arrow_marker_;
  inter_arrow_marker_.header.frame_id = target_frame_;
  inter_arrow_marker_.scale = ARROW_INTERACTIVE_SCALE;
  // Convert tf2::Transform to geometry_msgs::msg::Pose
  inter_arrow_marker_.pose.position.x = box_pos_.getOrigin().x();
  inter_arrow_marker_.pose.position.y = box_pos_.getOrigin().y();
  inter_arrow_marker_.pose.position.z = box_pos_.getOrigin().z();
  inter_arrow_marker_.pose.orientation.x = box_pos_.getRotation().x();
  inter_arrow_marker_.pose.orientation.y = box_pos_.getRotation().y();
  inter_arrow_marker_.pose.orientation.z = box_pos_.getRotation().z();
  inter_arrow_marker_.pose.orientation.w = box_pos_.getRotation().w();
  inter_arrow_marker_.description = "Interaction Marker";
  inter_arrow_marker_.name = "add_point_button";

  makeInteractiveMarkerControl(inter_arrow_marker_);
  server_->insert(inter_arrow_marker_);
  server_->setCallback(inter_arrow_marker_.name, std::bind(&BasePlacementPanel::processFeedback, this, std::placeholders::_1));
}

void BasePlacementPanel::parseWayPoints()
{
  geometry_msgs::msg::Pose target_pose;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  for (size_t i = 0; i < waypoints_pos_.size(); ++i)
  {
    // Convert tf2::Transform to geometry_msgs::msg::Pose
    target_pose.position.x = waypoints_pos_[i].getOrigin().x();
    target_pose.position.y = waypoints_pos_[i].getOrigin().y();
    target_pose.position.z = waypoints_pos_[i].getOrigin().z();
    target_pose.orientation.x = waypoints_pos_[i].getRotation().x();
    target_pose.orientation.y = waypoints_pos_[i].getRotation().y();
    target_pose.orientation.z = waypoints_pos_[i].getRotation().z();
    target_pose.orientation.w = waypoints_pos_[i].getRotation().w();
    waypoints.push_back(target_pose);
  }

  if (waypoints.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "No waypoints to process!");
    return;
  }

  // Emit signal for backward compatibility
  Q_EMIT wayPoints_signal(waypoints);

  // Wait for action server to be available
  if (!find_base_action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    return;
  }

  // Emit signal to indicate processing started
  Q_EMIT basePlacementProcessStarted();

  // Create and send goal
  auto goal_msg = FindBaseAction::Goal();

  // Convert Pose vector to PoseNamed vector
  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    base_placement_interfaces::msg::PoseNamed named_pose;
    named_pose.name = "task_pose_" + std::to_string(i);
    named_pose.pose = waypoints[i];
    goal_msg.task_poses.push_back(named_pose);
  }

  goal_msg.method_index = selected_method_;
  goal_msg.num_base_locations = num_base_locations_;
  goal_msg.num_high_score_spheres = num_high_score_spheres_;
  goal_msg.use_provided_spheres = false;
  goal_msg.use_provided_union_map = false;

  // Set up send options with callbacks
  auto send_goal_options = rclcpp_action::Client<FindBaseAction>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&BasePlacementPanel::goalResponseCallback, this, std::placeholders::_1);

  send_goal_options.feedback_callback =
    std::bind(&BasePlacementPanel::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback =
    std::bind(&BasePlacementPanel::resultCallback, this, std::placeholders::_1);

  // Send the goal asynchronously
  RCLCPP_INFO(node_->get_logger(),
    "Sending goal to base_placement_server: method=%d, locations=%d, spheres=%d",
    selected_method_, num_base_locations_, num_high_score_spheres_);

  find_base_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void BasePlacementPanel::saveWayPointsToFile()
{
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save Way Points"), ".yaml", tr("Way Points (*.yaml);;All Files (*)"));
  if (fileName.isEmpty()) return;

  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly))
  {
    QMessageBox::information(this, tr("Unable to open file"), file.errorString());
    file.close();
    return;
  }

  YAML::Emitter out;
  out << YAML::BeginSeq;

  for (size_t i = 0; i < waypoints_pos_.size(); ++i)
  {
    out << YAML::BeginMap;
    std::vector<double> points_vec;
    points_vec.push_back(waypoints_pos_[i].getOrigin().x());
    points_vec.push_back(waypoints_pos_[i].getOrigin().y());
    points_vec.push_back(waypoints_pos_[i].getOrigin().z());

    double rx, ry, rz;
    tf2::Matrix3x3 m(waypoints_pos_[i].getRotation());
    m.getRPY(rx, ry, rz);
    points_vec.push_back(rx * 180.0 / M_PI);
    points_vec.push_back(ry * 180.0 / M_PI);
    points_vec.push_back(rz * 180.0 / M_PI);

    out << YAML::Key << "name";
    out << YAML::Value << (i + 1);
    out << YAML::Key << "point";
    out << YAML::Value << YAML::Flow << points_vec;
    out << YAML::EndMap;
  }

  out << YAML::EndSeq;

  std::ofstream myfile;
  myfile.open(fileName.toStdString());
  myfile << out.c_str();
  myfile.close();
  file.close();
}

void BasePlacementPanel::clearAllPointsRViz()
{
  waypoints_pos_.clear();
  server_->clear();
  count_ = 0;
  makeInteractiveMarker();
  server_->applyChanges();
}

void BasePlacementPanel::getRobotModelFrame_slot(const tf2::Transform end_effector)
{
  target_frame_.assign("base_link");
  RCLCPP_INFO_STREAM(rclcpp::get_logger("base_placement_plugin"), "The robot model frame is: " << target_frame_);
  box_pos_ = end_effector;
  clearAllPointsRViz();
  count_ = 0;
  makeInteractiveMarker();
  server_->applyChanges();
}

// ============================================================
// NEW SLOTS FOR HANDLING WIDGET INPUTS
// ============================================================

void BasePlacementPanel::setBasePlaceParams(int num_base_locations, int num_high_score_spheres)
{
  num_base_locations_ = num_base_locations;
  num_high_score_spheres_ = num_high_score_spheres;

  RCLCPP_INFO(node_->get_logger(),
    "Base placement parameters updated: locations=%d, spheres=%d",
    num_base_locations, num_high_score_spheres);

  // Optionally update server parameters via service
  if (update_parameters_client_->wait_for_service(std::chrono::seconds(1)))
  {
    auto request = std::make_shared<base_placement_interfaces::srv::UpdateParameters::Request>();
    request->method_index = selected_method_;
    request->num_base_locations = num_base_locations;
    request->num_high_score_spheres = num_high_score_spheres;
    update_parameters_client_->async_send_request(request);
  }
}

void BasePlacementPanel::loadReachabilityFromFiles(QString irm_file_path, QString rm_file_path)
{
  RCLCPP_INFO(node_->get_logger(), "Loading reachability data from files via server...");
  RCLCPP_INFO(node_->get_logger(), "  IRM file: %s", irm_file_path.toStdString().c_str());
  if (!rm_file_path.isEmpty()) {
    RCLCPP_INFO(node_->get_logger(), "  RM file: %s", rm_file_path.toStdString().c_str());
  }

  // Wait for service to be available
  if (!update_reachability_client_->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "Service 'update_reachability_map' not available after 5 seconds. "
                 "Is the BasePlacementServer running?");
    QMessageBox::critical(nullptr, "Service Error",
                         "Cannot connect to BasePlacementServer.\n"
                         "Please ensure the server node is running.");
    return;
  }

  // Create service request with file paths
  auto request = std::make_shared<base_placement_interfaces::srv::UpdateReachabilityMap::Request>();
  request->irm_file_path = irm_file_path.toStdString();
  request->rm_file_path = rm_file_path.toStdString();
  request->load_irm = true;
  request->load_rm = !rm_file_path.isEmpty();

  RCLCPP_INFO(node_->get_logger(), "Sending service request to server...");

  // Use a lambda to handle the response
  auto response_callback = [this](rclcpp::Client<base_placement_interfaces::srv::UpdateReachabilityMap>::SharedFuture future)
  {
    try {
      auto response = future.get();

      if (response->success)
      {
        RCLCPP_INFO(node_->get_logger(),
                    "Server successfully loaded reachability data: %d spheres, resolution=%.4f m",
                    response->num_spheres_loaded, response->resolution);

        // Update local cache for backward compatibility
        resolution_ = response->resolution;

        QMessageBox::information(nullptr, "Success",
                                QString("Reachability data loaded successfully!\n"
                                       "Spheres: %1\n"
                                       "Resolution: %2 m")
                                .arg(response->num_spheres_loaded)
                                .arg(response->resolution));
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Server failed to load reachability data: %s",
                     response->message.c_str());

        QMessageBox::critical(nullptr, "Loading Error",
                             QString("Failed to load reachability data:\n%1")
                             .arg(QString::fromStdString(response->message)));
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Exception while waiting for service response: %s", e.what());

      QMessageBox::critical(nullptr, "Service Error",
                           QString("Error communicating with server:\n%1")
                           .arg(e.what()));
    }
  };

  // Send async request with callback
  update_reachability_client_->async_send_request(request, response_callback);
}

void BasePlacementPanel::setSelectedMethod(int method_index)
{
  selected_method_ = method_index;
  RCLCPP_INFO(node_->get_logger(), "Selected method: %d", method_index);
}

void BasePlacementPanel::setSelectedOpType(int output_type_index)
{
  selected_output_type_ = output_type_index;
  RCLCPP_INFO(node_->get_logger(), "Selected output type: %d", output_type_index);
}

void BasePlacementPanel::setUserBasePoses(const std::vector<geometry_msgs::msg::Pose>& poses)
{
  user_base_poses_ = poses;
  RCLCPP_INFO(node_->get_logger(), "User-defined base poses set: %zu poses", poses.size());
}

// ============================================================
// ACTION CLIENT CALLBACKS
// ============================================================

void BasePlacementPanel::goalResponseCallback(const GoalHandleFindBase::SharedPtr& goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    Q_EMIT basePlacementProcessFinished();
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    goal_handle_ = goal_handle;
  }
}

void BasePlacementPanel::feedbackCallback(
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
}

void BasePlacementPanel::resultCallback(const GoalHandleFindBase::WrappedResult& result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Goal succeeded!");
      RCLCPP_INFO(node_->get_logger(),
        "Received %zu base poses with best score: %.2f, computation time: %.3fs",
        result.result->base_poses.size(),
        result.result->best_score,
        result.result->computation_time_seconds);

      // Emit signals to update GUI
      Q_EMIT basePlacementProcessCompleted(result.result->best_score);
      Q_EMIT basePlacementProcessFinished();
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

} // namespace base_placement_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(base_placement_plugin::BasePlacementPanel, rviz_common::Panel)
