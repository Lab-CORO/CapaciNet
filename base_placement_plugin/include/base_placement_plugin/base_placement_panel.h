#ifndef BASE_PLACEMENT_PANEL_H_
#define BASE_PLACEMENT_PANEL_H_

#ifndef Q_MOC_RUN
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <iterator>
#include <memory>
#include <functional>

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
#include <base_placement_plugin/widgets/base_placement_widget.h>
#include <base_placement_plugin/interactive_base_selector.h>

// ROS2 action and service includes
#include <rclcpp_action/rclcpp_action.hpp>
#include <base_placement_interfaces/action/find_base.hpp>
#include <base_placement_interfaces/srv/update_reachability_map.hpp>
#include <base_placement_interfaces/srv/update_parameters.hpp>

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

namespace base_placement_plugin
{
/*!
 *  \brief     RViz Panel for interactive base placement configuration and visualization
 *  \details   The BasePlacementPanel provides a complete interface for base placement:
 *             - 3D interactive markers for waypoint definition
 *             - Integration with BasePlacementWidget for parameter configuration
 *             - Action client for communication with base placement server
 *             - Real-time feedback and result visualization in RViz
 *             This Class inherits from the rviz_common::Panel superclass.
 *  \author    Risto Kojcev (adapted to ROS2), Guillaume Dupoiron (refactored)
 */
class BasePlacementPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  //! A Constructor for the RViz Panel.
  BasePlacementPanel(QWidget* parent = 0);
  //! Virtual Destructor for the RViz Panel.
  virtual ~BasePlacementPanel();
  //! RViz panel initialization
  virtual void onInitialize();

  //! Function for all the interactive marker interactions
  virtual void processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

  //! Make a new Interactive Marker Way-Point
  virtual void makeArrow(const tf2::Transform& point_pos, int count_arrow);
  //! User Interaction Arrow Marker
  virtual void makeInteractiveMarker();

private:
  //! Function for creating a way-point marker
  visualization_msgs::msg::Marker makeWayPoint(
    visualization_msgs::msg::InteractiveMarker& msg);
  
  //! Function to create the InteractionArrow Marker
  visualization_msgs::msg::Marker makeInterArrow(
    visualization_msgs::msg::InteractiveMarker& msg);
  
  //! Create controls for default starting control ArrowMarkers
  visualization_msgs::msg::InteractiveMarkerControl& makeArrowControlDefault(
    visualization_msgs::msg::InteractiveMarker& msg);
  
  //! 6DOF control for the Interactive Markers
  visualization_msgs::msg::InteractiveMarkerControl& makeArrowControlDetails(
    visualization_msgs::msg::InteractiveMarker& msg);

  //! The box control can be used as a pointer to a certain 3D location
  visualization_msgs::msg::InteractiveMarkerControl& makeInteractiveMarkerControl(
    visualization_msgs::msg::InteractiveMarker& msg_box);
  
  //! Function to handle the entries made from the Way-Points interactive markers Menu.
  virtual void changeMarkerControlAndPose(std::string marker_name, bool set_control);

  //! Define a server for the Interactive Markers.
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;

  //! Vector for storing all the User Entered Way-Points.
  std::vector<tf2::Transform> waypoints_pos_;
  //! The position of the User handleable Interactive Marker.
  tf2::Transform box_pos_;

  //! Variable for storing the count of the Way-Points.
  int count_;
  //! Target Frame for the Transformation.
  std::string target_frame_;

  //! ROS2 node (shared with RViz)
  rclcpp::Node::SharedPtr node_;

protected Q_SLOTS:
  //! rviz_common::Panel virtual functions for loading Panel Configuration.
  virtual void load(const rviz_common::Config& config);
  //! rviz_common::Panel virtual functions for saving Panel Configuration.
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  //! Slot for handling the event of way-point deletion.
  virtual void pointDeleted(std::string marker_name);
  //! Slot for handling the add way-point event from the RQT UI.
  void addPointFromUI(const tf2::Transform point_pos);
  //! Slot for handling when the user updates the position of the Interactive Markers.
  void pointPoseUpdated(const tf2::Transform& point_pos, const char* marker_name);
  //! Slot for parsing the Way-Points before sending them to the action server.
  void parseWayPoints();
  //! Save all the Way-Points to a yaml file.
  void saveWayPointsToFile();
  //! Clear all the Way-Points
  void clearAllPointsRViz();
  //! Get the name of the Transformation frame of the Robot.
  void getRobotModelFrame_slot(const tf2::Transform end_effector);
  //! Set base placement parameters (from widget)
  void setBasePlaceParams(int num_base_locations, int num_high_score_spheres);
  //! Load reachability data from file paths (sends to server)
  void loadReachabilityFromFiles(QString irm_file_path, QString rm_file_path);
  //! Set selected method (from widget)
  void setSelectedMethod(int method_index);
  //! Set selected output type (from widget)
  void setSelectedOpType(int output_type_index);
  //! Set user-defined base poses (from widget)
  void setUserBasePoses(const std::vector<geometry_msgs::msg::Pose>& poses);

Q_SIGNALS:
  //! Signal for notifying that a way-point was deleted in the RViz environment.
  void pointDeleteRviz(int marker_name_nr);
  //! Signal for notifying that a way-point has been added from the RViz environment.
  void addPointRViz(const tf2::Transform& point_pos, const int count);
  //! Signal that the way-point position has been updated by the user from the RViz environment.
  void pointPoseUpdatedRViz(const tf2::Transform& point_pos, const char* marker_name);
  //! Signal for sending all the Way-Points.
  void wayPoints_signal(std::vector<geometry_msgs::msg::Pose> waypoints);
  //! Signals for communicating with widget
  void getinitialmarkerFrame_signal(const tf2::Transform frame);
  void basePlacementProcessStarted();
  void basePlacementProcessFinished();
  void basePlacementProcessCompleted(double score);
  void sendBasePlaceMethods_signal(std::vector<std::string> methods);
  void sendOuputType_signal(std::vector<std::string> output_types);

protected:
  //! The class that GUI RQT User Interactions.
  QWidget* widget_;

private:
  //! Action client for base placement
  using FindBaseAction = base_placement_interfaces::action::FindBase;
  using GoalHandleFindBase = rclcpp_action::ClientGoalHandle<FindBaseAction>;
  rclcpp_action::Client<FindBaseAction>::SharedPtr find_base_action_client_;
  rclcpp::CallbackGroup::SharedPtr action_client_cb_group_;
  GoalHandleFindBase::SharedPtr goal_handle_;

  //! Service clients
  rclcpp::Client<base_placement_interfaces::srv::UpdateReachabilityMap>::SharedPtr update_reachability_client_;
  rclcpp::Client<base_placement_interfaces::srv::UpdateParameters>::SharedPtr update_parameters_client_;

  //! Parameters
  int selected_method_;
  int selected_output_type_;
  int num_base_locations_;
  int num_high_score_spheres_;
  std::vector<geometry_msgs::msg::Pose> user_base_poses_;

  //! Reachability data storage
  std::multimap<std::vector<double>, std::vector<double>> pose_col_filter_;
  std::multimap<std::vector<double>, double> sphere_col_;
  float resolution_;

  //! Action callbacks
  void goalResponseCallback(const GoalHandleFindBase::SharedPtr& goal_handle);
  void feedbackCallback(GoalHandleFindBase::SharedPtr,
                       const std::shared_ptr<const FindBaseAction::Feedback> feedback);
  void resultCallback(const GoalHandleFindBase::WrappedResult& result);

  //! Define constants for color, arrow size, etc.
  std_msgs::msg::ColorRGBA WAY_POINT_COLOR;
  std_msgs::msg::ColorRGBA ARROW_INTER_COLOR;

  geometry_msgs::msg::Vector3 WAY_POINT_SCALE_CONTROL;
  geometry_msgs::msg::Vector3 ARROW_INTER_SCALE_CONTROL;

  float INTERACTIVE_MARKER_SCALE;
  float ARROW_INTERACTIVE_SCALE;
};

}  // end of namespace base_placement_plugin

#endif  // BASE_PLACEMENT_PANEL_H_