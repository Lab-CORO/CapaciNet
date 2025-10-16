#ifndef ADD_ROBOT_BASE_H
#define ADD_ROBOT_BASE_H

#ifndef Q_MOC_RUN
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <iterator>
#include <memory>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/string_property.hpp>
// #include <base_placement_plugin/widgets/base_placement_widget.h>
// #include <base_placement_plugin/place_base.h>
#include <base_placement_plugin/create_marker.h>

#include <QWidget>
#include <QCursor>
#include <QObject>
#include <QKeyEvent>
#include <QHBoxLayout>
#include <QTimer>
#include <QtConcurrent>
#include <QFuture>

#endif

namespace rviz_common
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class StringProperty;
class BoolProperty;
}

class AddRobotBase : public QObject
{
  Q_OBJECT

public:
  AddRobotBase(std::shared_ptr<rclcpp::Node> node, QWidget* parent, std::string group_name);
  virtual ~AddRobotBase();
  void init();

  virtual void processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

  void changeMarkerControlAndPose(std::string marker_name, bool set_control);
  
  visualization_msgs::msg::InteractiveMarkerControl& makeArrowControlDefault(
    visualization_msgs::msg::InteractiveMarker& msg);
  
  visualization_msgs::msg::InteractiveMarkerControl& makeArrowControlDetails(
    visualization_msgs::msg::InteractiveMarker& msg);
  
  visualization_msgs::msg::Marker makeWayPoint(
    visualization_msgs::msg::InteractiveMarker& msg);
  
  void pointDeleted(std::string marker_name);
  void makeArrow(const tf2::Transform& point_pos, int count_arrow);
  void makeInteractiveMarker();
  
  visualization_msgs::msg::InteractiveMarkerControl& makeInteractiveMarkerControl(
    visualization_msgs::msg::InteractiveMarker& msg);
  
  visualization_msgs::msg::Marker makeInterArrow(
    visualization_msgs::msg::InteractiveMarker& msg);
  
  visualization_msgs::msg::MarkerArray makeRobotMarker(
    visualization_msgs::msg::InteractiveMarker& msg, bool waypoint);

  void pointPoseUpdated(const tf2::Transform& point_pos, const char* marker_name);
  void getWaypoints(std::vector<geometry_msgs::msg::Pose>& waypoints);

public Q_SLOTS:
  void parseWayPoints();
  void clearAllPointsRviz();
  void getRobotModelFrame_slot(const tf2::Transform end_effector);

protected:
  QWidget* widget_;

Q_SIGNALS:
  void baseWayPoints_signal(std::vector<geometry_msgs::msg::Pose> base_poses);

private:
  // ROS2 node
  std::shared_ptr<rclcpp::Node> node_;
  
  // Interactive marker server
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;

  // Color and scale properties
  std_msgs::msg::ColorRGBA WAY_POINT_COLOR;
  std_msgs::msg::ColorRGBA ROBOT_WAY_POINT_COLOR;
  std_msgs::msg::ColorRGBA ARROW_INTER_COLOR;
  std_msgs::msg::ColorRGBA ROBOT_INTER_COLOR;
  geometry_msgs::msg::Vector3 WAY_POINT_SCALE_CONTROL;
  geometry_msgs::msg::Vector3 ARROW_INTER_SCALE_CONTROL;

  float INTERACTIVE_MARKER_SCALE;
  float ARROW_INTERACTIVE_SCALE;

  // Waypoints storage
  std::vector<tf2::Transform> waypoints_pos_;
  tf2::Transform box_pos_;
  int count_;
  std::string target_frame_;

  // Robot visualization
  visualization_msgs::msg::MarkerArray robot_markers_;
  std::shared_ptr<CreateMarker> mark_;
  std::string group_name_;
};

#endif // ADD_ROBOT_BASE_H