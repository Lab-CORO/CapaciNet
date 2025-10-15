// add_way_point_ros2.cpp
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <fstream>

#include <QHBoxLayout>
#include <QFileDialog>
#include <QMessageBox>

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/panel.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <yaml-cpp/yaml.h>

// includes de ton UI et des classes PlaceBase/widgets déjà existantes
#include "base_placement_plugin/add_way_point.h" // adapte si le header est renommé

namespace base_placement_plugin
{

AddWayPoint::AddWayPoint(QWidget* parent)
: rviz_common::Panel(parent),
  node_(nullptr),
  count_(0)
{
  setObjectName("BasePlacementPlannerPlugin");

  // Créer un node rclcpp local pour l'interactive marker server_.
  // IMPORTANT: rclcpp::init() doit déjà avoir été appelé par l'application (rviz2).
  node_ = rclcpp::Node::make_shared("base_placement_plugin_rviz_panel");

  // NOTE: La signature du constructeur InteractiveMarkerServer varie selon la distro.
  // Certains constructeurs attendent (node, name) ; certains (name, options).
  // Essayez cette forme; si la compilation échoue, adaptez la signature selon votre distro.
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("marker_server", node_);

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

AddWayPoint::~AddWayPoint()
{
  server_.reset();
}

void AddWayPoint::onInitialize()
{
  place_base_ = new PlaceBase(node_);

  widget_ = new widgets::BasePlacementWidget("~");
  this->parentWidget()->resize(widget_->width(), widget_->height());
  QHBoxLayout* main_layout = new QHBoxLayout(this);
  main_layout->addWidget(widget_);

  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "initializing..");

  menu_handler_.insert("Delete", std::bind(&AddWayPoint::processFeedback, this, _1));
  {
    auto h = menu_handler_.insert("Fine adjustment", std::bind(&AddWayPoint::processFeedback, this, _1));
    menu_handler_.setCheckState(h, interactive_markers::MenuHandler::UNCHECKED);
  }

  // connexions Qt: remplacer tf::Transform par tf2::Transform
  connect(place_base_, SIGNAL(getinitialmarkerFrame_signal(const tf2::Transform)), this,
          SLOT(getRobotModelFrame_slot(const tf2::Transform)));

  connect(place_base_, SIGNAL(getinitialmarkerFrame_signal(const tf2::Transform)), widget_,
          SLOT(setAddPointUIStartPos(const tf2::Transform)));
  connect(place_base_, SIGNAL(basePlacementProcessStarted()), widget_, SLOT(PlaceBaseStartedHandler()));
  connect(place_base_, SIGNAL(basePlacementProcessFinished()), widget_, SLOT(PlaceBaseFinishedHandler()));
  connect(place_base_, SIGNAL(basePlacementProcessCompleted(double)), widget_, SLOT(PlaceBaseCompleted_slot(double)));
  connect(place_base_, SIGNAL(sendBasePlaceMethods_signal(std::vector< std::string >)), widget_,
          SLOT(getBasePlacePlanMethod(std::vector< std::string >)));
  connect(place_base_, SIGNAL(sendOuputType_signal(std::vector< std::string >)), widget_,
          SLOT(getOutputType(std::vector< std::string >)));
  connect(place_base_, SIGNAL(sendGroupType_signal(std::vector< std::string >)), widget_,
          SLOT(getRobotGroups(std::vector< std::string >)));
  connect(place_base_, SIGNAL(sendSelectedGroup_signal(std::string)), widget_,
          SLOT(getSelectedGroup(std::string)));

  // Widget -> PlaceBase
  connect(widget_, SIGNAL(basePlacementParamsFromUI_signal(int, int)), place_base_, SLOT(setBasePlaceParams(int, int)));
  connect(widget_, SIGNAL(reachabilityData_signal(std::multimap< std::vector< double >, std::vector< double > >,
                                                  std::multimap< std::vector< double >, double >, float)),
          place_base_, SLOT(setReachabilityData(std::multimap< std::vector< double >, std::vector< double > >,
                                               std::multimap< std::vector< double >, double >, float)));
  connect(widget_, SIGNAL(showUnionMap_signal(bool)), place_base_, SLOT(ShowUnionMap(bool)));
  connect(widget_, SIGNAL(clearUnionMap_signal(bool)), place_base_, SLOT(clearUnionMap(bool)));
  connect(widget_, SIGNAL(SendSelectedMethod(int)), place_base_, SLOT(getSelectedMethod(int)));
  connect(widget_, SIGNAL(SendSelectedOpType(int)), place_base_, SLOT(getSelectedOpType(int)));
  connect(widget_, SIGNAL(SendSelectedRobotGroup(int)), place_base_, SLOT(getSelectedRobotGroup(int)));
  connect(widget_, SIGNAL(SendShowUmodel(bool)), place_base_, SLOT(getShowUreachModels(bool)));
  connect(widget_, SIGNAL(SendBasePoses(std::vector<geometry_msgs::msg::Pose>)), place_base_, SLOT(getBasePoses(std::vector<geometry_msgs::msg::Pose>)));

  // Widget -> this
  connect(widget_, SIGNAL(addPoint(tf2::Transform)), this, SLOT(addPointFromUI(tf2::Transform)));
  connect(widget_, SIGNAL(pointDelUI_signal(std::string)), this, SLOT(pointDeleted(std::string)));
  connect(widget_, SIGNAL(parseWayPointBtn_signal()), this, SLOT(parseWayPoints()));
  connect(widget_, SIGNAL(pointPosUpdated_signal(const tf2::Transform&, const char*)), this,
          SLOT(pointPoseUpdated(const tf2::Transform&, const char*)));
  connect(widget_, SIGNAL(saveToFileBtn_press()), this, SLOT(saveWayPointsToFile()));
  connect(widget_, SIGNAL(clearAllPoints_signal()), this, SLOT(clearAllPointsRViz()));

  // this -> widget
  connect(this, SIGNAL(addPointRViz(const tf2::Transform&, const int)), widget_,
          SLOT(insertRow(const tf2::Transform&, const int)));
  connect(this, SIGNAL(pointPoseUpdatedRViz(const tf2::Transform&, const char*)), widget_,
          SLOT(pointPosUpdated_slot(const tf2::Transform&, const char*)));
  connect(this, SIGNAL(pointDeleteRviz(int)), widget_, SLOT(removeRow(int)));

  // this -> place_base_
  connect(this, SIGNAL(wayPoints_signal(std::vector< geometry_msgs::msg::Pose >)), place_base_,
          SLOT(BasePlacementHandler(std::vector< geometry_msgs::msg::Pose >)));
  connect(this, SIGNAL(initRviz()), place_base_, SLOT(initRvizDone()));

  Q_EMIT initRviz();
  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "ready.");
}

void AddWayPoint::load(const rviz_common::Config& config)
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

void AddWayPoint::save(rviz_common::Config config) const
{
  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "Saving configuration");
  rviz_common::Panel::save(config);
  config.mapSetValue("TextEntry", QString::fromStdString(std::string("test_field")));
}

void AddWayPoint::addPointFromUI(const tf2::Transform point_pos)
{
  RCLCPP_INFO(rclcpp::get_logger("base_placement_plugin"), "Point Added");
  makeArrow(point_pos, count_);
  server_->applyChanges();
}

void AddWayPoint::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback)
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

void AddWayPoint::pointPoseUpdated(const tf2::Transform& point_pos, const char* marker_name)
{
  geometry_msgs::msg::Pose pose = tf2::toMsg(point_pos);
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

visualization_msgs::msg::Marker AddWayPoint::makeWayPoint(visualization_msgs::msg::InteractiveMarker& msg)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.scale = WAY_POINT_SCALE_CONTROL;
  marker.color = WAY_POINT_COLOR;
  // ROS2 msg action field not present on marker (was in visualization_msgs/Marker in ROS1)
  // marker.action = visualization_msgs::Marker::ADD; // not necessary
  return marker;
}

visualization_msgs::msg::InteractiveMarkerControl& AddWayPoint::makeArrowControlDefault(visualization_msgs::msg::InteractiveMarker& msg)
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

visualization_msgs::msg::InteractiveMarkerControl& AddWayPoint::makeArrowControlDetails(visualization_msgs::msg::InteractiveMarker& msg)
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

void AddWayPoint::makeArrow(const tf2::Transform& point_pos, int count_arrow)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("base_placement_plugin"), "Markers frame is: " << target_frame_);
  int_marker.header.frame_id = target_frame_;
  int_marker.scale = INTERACTIVE_MARKER_SCALE;
  int_marker.pose = tf2::toMsg(point_pos);

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
  server_->setCallback(int_marker.name, std::bind(&AddWayPoint::processFeedback, this, _1));
  menu_handler_.apply(*server_, int_marker.name);
}

void AddWayPoint::changeMarkerControlAndPose(std::string marker_name, bool set_control)
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
  server_->setCallback(int_marker.name, std::bind(&AddWayPoint::processFeedback, this, _1));
}

void AddWayPoint::pointDeleted(std::string marker_name)
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

visualization_msgs::msg::Marker AddWayPoint::makeInterArrow(visualization_msgs::msg::InteractiveMarker& msg)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.scale = ARROW_INTER_SCALE_CONTROL;
  marker.color = ARROW_INTER_COLOR;
  return marker;
}

visualization_msgs::msg::InteractiveMarkerControl& AddWayPoint::makeInteractiveMarkerControl(visualization_msgs::msg::InteractiveMarker& msg)
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

void AddWayPoint::makeInteractiveMarker()
{
  visualization_msgs::msg::InteractiveMarker inter_arrow_marker_;
  inter_arrow_marker_.header.frame_id = target_frame_;
  inter_arrow_marker_.scale = ARROW_INTERACTIVE_SCALE;
  inter_arrow_marker_.pose = tf2::toMsg(box_pos_);
  inter_arrow_marker_.description = "Interaction Marker";
  inter_arrow_marker_.name = "add_point_button";

  makeInteractiveMarkerControl(inter_arrow_marker_);
  server_->insert(inter_arrow_marker_);
  server_->setCallback(inter_arrow_marker_.name, std::bind(&AddWayPoint::processFeedback, this, _1));
}

void AddWayPoint::parseWayPoints()
{
  geometry_msgs::msg::Pose target_pose;
  std::vector< geometry_msgs::msg::Pose > waypoints;
  for (size_t i = 0; i < waypoints_pos_.size(); ++i)
  {
    target_pose = tf2::toMsg(waypoints_pos_[i]);
    waypoints.push_back(target_pose);
  }
  Q_EMIT wayPoints_signal(waypoints);
}

void AddWayPoint::saveWayPointsToFile()
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

void AddWayPoint::clearAllPointsRViz()
{
  waypoints_pos_.clear();
  server_->clear();
  count_ = 0;
  makeInteractiveMarker();
  server_->applyChanges();
}

void AddWayPoint::getRobotModelFrame_slot(const tf2::Transform end_effector)
{
  target_frame_.assign("base_link");
  RCLCPP_INFO_STREAM(rclcpp::get_logger("base_placement_plugin"), "The robot model frame is: " << target_frame_);
  box_pos_ = end_effector;
  clearAllPointsRViz();
  count_ = 0;
  makeInteractiveMarker();
  server_->applyChanges();
}

} // namespace base_placement_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(base_placement_plugin::AddWayPoint, rviz_common::Panel)
