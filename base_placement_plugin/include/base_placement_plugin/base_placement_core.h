#ifndef BASE_PLACEMENT_CORE_H_
#define BASE_PLACEMENT_CORE_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <base_placement_plugin/sphere_discretization.h>
#include "curobo_msgs/srv/ik.hpp"

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>

/*!
 *  \brief     Core computation class for Base Placement (no Qt, no RViz)
 *  \details   Pure C++ class that handles all base placement algorithms
 *             Designed to be used by both RViz plugin and standalone ROS2 node
 *  \author    Guillaume Dupoiron (refactored from PlaceBase)
 */
class BasePlacementCore
{
public:
  //! Workspace Sphere structure
  struct WsSphere {
    std_msgs::msg::Header header;
    geometry_msgs::msg::Point point;
    float ri;
    std::vector<geometry_msgs::msg::Pose> poses;
  };

  //! Workspace structure
  struct WorkSpace {
    std_msgs::msg::Header header;
    float resolution;
    std::vector<BasePlacementCore::WsSphere> ws_spheres;
  };

  //! Named pose structure
  struct NamedPose {
    std::string name;
    geometry_msgs::msg::Pose pose;
  };

  //! Base placement method enumeration
  enum class Method {
    PCA = 0,
    GraspReachabilityScore = 1,
    IKSolutionScore = 2,
    VerticalRobotModel = 3,
    UserIntuition = 4
  };

  //! Computation result structure
  struct ComputationResult {
    bool success;
    std::string message;
    std::vector<geometry_msgs::msg::Pose> base_poses;
    std::vector<double> scores;
    double best_score;
    int best_index;
    double computation_time_seconds;
  };

  //! Feedback callback type for iterative computation
  using FeedbackCallback = std::function<void(
    const std::string& phase,
    int iteration,
    int total_iterations,
    double progress_percentage,
    const std::string& status_message,
    int candidates_evaluated,
    double current_best_score
  )>;

  //! Constructor
  BasePlacementCore(std::shared_ptr<rclcpp::Node> node);

  //! Virtual Destructor
  virtual ~BasePlacementCore();

  //! Initialize the core
  void init();

  // ============================================================
  // CONFIGURATION METHODS
  // ============================================================

  //! Set base placement parameters
  void setParameters(Method method, int num_base_locations, int num_high_score_spheres);

  //! Set the selected method
  void setMethod(Method method);

  //! Set number of base locations to compute
  void setNumBaseLocations(int num);

  //! Set number of high score spheres
  void setNumHighScoreSpheres(int num);

  //! Set pre-computed high-score spheres (from client)
  void setHighScoreSpheres(const std::vector<std::vector<double>>& spheres);

  //! Set pre-computed union map (from client)
  void setBaseTrnsCol(const std::multimap<std::vector<double>, std::vector<double>>& base_trns_col);

  // ============================================================
  // DATA LOADING METHODS
  // ============================================================

  //! Load reachability data from memory
  bool setReachabilityData(
    std::multimap<std::vector<double>, std::vector<double>> pose_collection,
    std::multimap<std::vector<double>, double> sphere_collection,
    float resolution
  );

  //! Load reachability data from HDF5 files
  bool loadReachabilityFromFile(
    const std::string& irm_file_path,
    const std::string& rm_file_path = ""
  );

  // ============================================================
  // POSE MANAGEMENT METHODS
  // ============================================================

  //! Add a named task pose
  bool addNamedPose(const std::string& name, const geometry_msgs::msg::Pose& pose);

  //! Remove a named pose by name
  bool removeNamedPose(const std::string& name);

  //! Clear all task poses
  void clearTaskPoses();

  //! Get all task poses
  std::vector<NamedPose> getTaskPoses() const;

  //! Set task poses (replaces all)
  void setTaskPoses(const std::vector<geometry_msgs::msg::Pose>& poses);

  //! Set user-defined base poses (for UserIntuition method)
  void setUserBasePoses(const std::vector<geometry_msgs::msg::Pose>& poses);

  // ============================================================
  // COMPUTATION METHODS
  // ============================================================

  //! Main computation method - find optimal base locations
  ComputationResult findBasePlacements(
    const std::vector<geometry_msgs::msg::Pose>& task_poses,
    FeedbackCallback feedback_callback = nullptr
  );

  //! Get union map (combined reachability from current task poses)
  WorkSpace getUnionMap(bool compute_from_current_poses = true);

  // ============================================================
  // RESULT RETRIEVAL METHODS
  // ============================================================

  //! Get last computed base poses
  std::vector<geometry_msgs::msg::Pose> getComputedBasePoses() const;

  //! Get last computation scores
  std::vector<double> getComputedScores() const;

  //! Get best score from last computation
  double getBestScore() const;

  //! Get best pose from last computation
  geometry_msgs::msg::Pose getBestPose() const;

  // ============================================================
  // UTILITY METHODS
  // ============================================================

  //! Clear all data (reachability, union map, computed bases)
  void clearAllData();

  //! Clear union map
  void clearUnionMap();

  //! Clear reachability data
  void clearReachabilityData();

  //! Get available method names
  static std::vector<std::string> getMethodNames();

  //! Get method name from index
  static std::string getMethodName(Method method);

protected:
  // ============================================================
  // ALGORITHM IMPLEMENTATIONS
  // ============================================================

  //! Base Placement by PCA
  ComputationResult findBaseByPCA(
    const std::vector<geometry_msgs::msg::Pose>& task_poses,
    FeedbackCallback feedback_callback
  );

  //! Base Placement by GraspReachabilityScore
  ComputationResult findBaseByGraspReachabilityScore(
    const std::vector<geometry_msgs::msg::Pose>& task_poses,
    FeedbackCallback feedback_callback
  );

  //! Base Placement by IKSolutionScore
  ComputationResult findBaseByIKSolutionScore(
    const std::vector<geometry_msgs::msg::Pose>& task_poses,
    FeedbackCallback feedback_callback
  );

  //! Base Placement by VerticalRobotModel
  ComputationResult findBaseByVerticalRobotModel(
    const std::vector<geometry_msgs::msg::Pose>& task_poses,
    FeedbackCallback feedback_callback
  );

  //! Base Placement by UserIntuition
  ComputationResult findBaseByUserIntuition(
    const std::vector<geometry_msgs::msg::Pose>& task_poses,
    FeedbackCallback feedback_callback
  );

  // ============================================================
  // HELPER METHODS
  // ============================================================

  //! Transform reachability data from arm base to robot base coordinates
  void transformToRobotbase(
    std::multimap<std::vector<double>, std::vector<double>> armBasePoses,
    std::multimap<std::vector<double>, std::vector<double>>& robotBasePoses
  );

  //! Transform from robot base to arm base
  void transformFromRobotbaseToArmBase(
    const geometry_msgs::msg::Pose& base_pose,
    geometry_msgs::msg::Pose& arm_base_pose
  );

  //! Create spheres for visualization and scoring
  void createSpheres(
    std::multimap<std::vector<double>, std::vector<double>> basePoses,
    std::map<std::vector<double>, double>& spColor,
    std::vector<std::vector<double>>& highScoredSp,
    bool reduce_D
  );

  //! Calculate score for robot base pose
  double calculateScoreForRobotBase(
    std::vector<geometry_msgs::msg::Pose>& grasp_poses,
    std::vector<geometry_msgs::msg::Pose>& base_poses
  );

  //! Calculate score for arm base pose
  double calculateScoreForArmBase(
    std::vector<geometry_msgs::msg::Pose>& grasp_poses,
    std::vector<geometry_msgs::msg::Pose>& base_poses
  );

  //! Check IK success with transformed base
  bool isIkSuccesswithTransformedBase(
    const geometry_msgs::msg::Pose& base_pose,
    const geometry_msgs::msg::Pose& grasp_pose,
    std::vector<double>& joint_soln,
    int& numOfSolns
  );

  // ============================================================
  // MEMBER VARIABLES
  // ============================================================

  //! ROS2 node
  std::shared_ptr<rclcpp::Node> node_;

  //! IK service client
  rclcpp::Client<curobo_msgs::srv::Ik>::SharedPtr client_ik_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  //! Configuration parameters
  Method selected_method_;
  int num_base_locations_;
  int num_high_score_spheres_;

  //! Reachability data
  std::multimap<std::vector<double>, std::vector<double>> pose_col_filter_;
  std::multimap<std::vector<double>, double> sphere_col_;
  float resolution_;

  //! Union map data
  std::multimap<std::vector<double>, std::vector<double>> base_trns_col_;
  std::map<std::vector<double>, double> sphere_color_;
  std::vector<std::vector<double>> high_score_sp_;

  //! Robot base pose collection
  std::multimap<std::vector<double>, std::vector<double>> robot_pose_col_filter_;

  //! Task poses (named)
  std::map<std::string, geometry_msgs::msg::Pose> named_task_poses_;

  //! Computation results
  std::vector<geometry_msgs::msg::Pose> computed_base_poses_;
  std::vector<double> computed_scores_;
  double best_score_;
  geometry_msgs::msg::Pose best_pose_;

  //! User-defined base poses for UserIntuition method
  std::vector<geometry_msgs::msg::Pose> user_base_poses_;

  //! Method names
  static const std::vector<std::string> method_names_;
};

#endif  // BASE_PLACEMENT_CORE_H_
