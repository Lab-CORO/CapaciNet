#ifndef BASE_PLACEMENT_WIDGET_H_
#define BASE_PLACEMENT_WIDGET_H_

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>

#include <ui_base_placement_widget.h>

#include <base_placement_plugin/base_placement_panel.h>
#include <base_placement_plugin/interactive_base_selector.h>

#include <QWidget>
#include <QTimer>
#include <QtConcurrentRun>
#include <QMainWindow>
#include <QTreeView>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QSplitter>
#include <QHeaderView>
#include <QCompleter>
#include <QIntValidator>
#include <QDataStream>
#include <QString>
#include <QFileDialog>
#include <QMessageBox>
#include <QProgressBar>

// macros
#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.29578)
#endif

typedef std::multimap<const std::vector<double>*, const std::vector<double>*> MultiMapPtr;
typedef std::map<const std::vector<double>*, double> MapVecDoublePtr;
typedef std::multimap<std::vector<double>, std::vector<double>> MultiMap;
typedef std::map<std::vector<double>, double> MapVecDouble;
typedef std::vector<std::vector<double>> VectorOfVectors;

namespace base_placement_plugin
{
namespace widgets
{

class BasePlacementWidget : public QWidget
{
  Q_OBJECT

public:
  explicit BasePlacementWidget(const std::string& ns = "", QWidget* parent = nullptr);

  // Setter pour le nœud ROS2 (appelé après construction)
  void setNode(std::shared_ptr<rclcpp::Node> node) { node_ = node; }

  virtual ~BasePlacementWidget();

  std::string get_name() const
  {
    return "BasePlacementPlanner";
  }

protected:
  void init();

  std::string param_ns_;
  Ui::BasePlacementWidget ui_;
  QStandardItemModel* pointDataModel = nullptr;

  InteractiveBaseSelector* add_robot = nullptr;

  bool show_union_map_ = false;
  bool show_umodels_ = false;
  std::string group_name_;
  std::shared_ptr<rclcpp::Node> node_;

private:
  QStringList pointList;

  void pointRange();

protected Q_SLOTS:
  void initTreeView();
  void pointDeletedUI();
  void pointAddUI();
  void insertRow(const tf2::Transform& point_pos, int count);
  void removeRow(int marker_nr);
  void pointPosUpdated_slot(const tf2::Transform& point_pos, const char* marker_name);
  void selectedPoint(const QModelIndex& current, const QModelIndex& previous);
  void treeViewDataChanged(const QModelIndex& index, const QModelIndex& index2);
  void parseWayPointBtn_slot();
  void savePointsToFile();
  void loadPointsFromFile();
  void loadReachabilityFiles();
  void showUnionMapFromUI();
  void clearUnionMapFromUI();
  void startUserIntution();
  void getWaypoints(const std::vector<geometry_msgs::msg::Pose>& base_poses);
  void clearAllPoints_slot();
  void setAddPointUIStartPos(const tf2::Transform& end_effector);
  void PlaceBaseStartedHandler();
  void PlaceBaseFinishedHandler();
  void sendBasePlacementParamsFromUI();
  void PlaceBaseCompleted_slot(double score);
  void getBasePlacePlanMethod(const std::vector<std::string>& methods);
  void getOutputType(const std::vector<std::string>& op_types);
  void getRobotGroups(const std::vector<std::string>& groups);
  void getSelectedGroup(const std::string& group_name);
  void selectedMethod(int index);
  void selectedOuputType(int op_index);
  void selectedRobotGroup(int index);
  void showUreachModels();

Q_SIGNALS:
  // Send file paths to panel (which forwards to server)
  void reachabilityFilePaths_signal(QString irm_file_path, QString rm_file_path);

  void showUnionMap_signal(bool show_union_map_);
  void clearUnionMap_signal(bool show_union_map_);
  void addPoint(const tf2::Transform point_pos);
  void pointDelUI_signal(const std::string& marker_name);
  void pointPosUpdated_signal(const tf2::Transform& position, const char* marker_name);
  void parseWayPointBtn_signal();
  void saveToFileBtn_press();
  void clearAllPoints_signal();
  void basePlacementParamsFromUI_signal(int base_loc_size_, int high_score_sp_);
  void SendSelectedMethod(int index);
  void SendSelectedOpType(int op_index);
  void SendSelectedRobotGroup(int index);
  void SendShowUmodel(bool umodel);
  void SendBasePoses(const std::vector<geometry_msgs::msg::Pose>& base_poses);

  //  Pour Qt signals avec types ROS2 personnalisés :
  // qRegisterMetaType<std::vector<geometry_msgs::msg::Pose>>("std::vector<geometry_msgs::msg::Pose>");
  // qRegisterMetaType<tf2::Transform>("tf2::Transform");
};

}  // namespace widgets
}  // namespace base_placement_plugin

#endif  // BASE_PLACEMENT_WIDGET_H_
