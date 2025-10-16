#include <base_placement_plugin/place_base.h>

#include <math.h>
#include <Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.hpp>

#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <visualization_msgs/msg/marker_array.hpp>

// #include <map_creator/WorkSpace.h>
#include "../include/base_placement_plugin/sphere_discretization.h"
// #include <map_creator/kinematics.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <visualization_msgs/msg/interactive_marker.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

PlaceBase::PlaceBase(std::shared_ptr<rclcpp::Node> node, QObject* parent)
  : QObject(parent), node_(node)
{
  init();
}

PlaceBase::~PlaceBase()
{
}

void PlaceBase::init()
{
  show_ureach_models_ = false;
  selected_method_ = 0;
  selected_op_type_ = 0;
  BASE_LOC_SIZE_ = 5;
  HIGH_SCORE_SP_ = 10;
  score_ = 0.0;
  res = 0.0f;
}

void PlaceBase::BasePlacementHandler(std::vector<geometry_msgs::msg::Pose> waypoints)
{
  RCLCPP_INFO(node_->get_logger(), "Starting concurrent process for Base Placement");
  QFuture<void> future = QtConcurrent::run(this, &PlaceBase::findbase, waypoints);
}

void PlaceBase::initRvizDone()
{
  RCLCPP_INFO(node_->get_logger(), "RViz is done now we need to emit the signal");

  tf2::Vector3 vec(0, 0, 0);
  tf2::Quaternion quat(0, 0, 0, 1);
  quat.normalize();
  tf2::Transform trns;
  trns.setOrigin(vec);
  trns.setRotation(quat);

  Q_EMIT getinitialmarkerFrame_signal(trns);

  RCLCPP_INFO(node_->get_logger(), "Sending the method list");
  method_names_.push_back("PrincipalComponentAnalysis");
  method_names_.push_back("GraspReachabilityScore");
  method_names_.push_back("IKSolutionScore");
  method_names_.push_back("VerticalRobotModel");
  method_names_.push_back("UserIntuition");
  Q_EMIT sendBasePlaceMethods_signal(method_names_);

  RCLCPP_INFO(node_->get_logger(), "Sending the output visualization method list");
  output_type_.push_back("Arrows");
  output_type_.push_back("Manipulator");
  output_type_.push_back("RobotModel");
  Q_EMIT sendOuputType_signal(output_type_);
}

void PlaceBase::getShowUreachModels(bool show_u_models)
{
  show_ureach_models_ = show_u_models;
  RCLCPP_INFO(node_->get_logger(), "Show unreachable model changing");
}

void PlaceBase::getSelectedRobotGroup(int group_index)
{
  if (group_index < 0 || group_index >= static_cast<int>(group_names_.size()))
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid group index: %d", group_index);
    return;
  }
  
  selected_group_ = group_names_[group_index];
  RCLCPP_INFO(node_->get_logger(), "Selected Group: %s", group_names_[group_index].c_str());
  
  mark_ = std::make_shared<CreateMarker>(node_, selected_group_);
  if (!mark_->checkEndEffector())
  {
    RCLCPP_ERROR(node_->get_logger(), "Is your selected group a manipulator??");
    mark_.reset();
  }
  Q_EMIT sendSelectedGroup_signal(selected_group_);
}

void PlaceBase::getSelectedMethod(int index)
{
  selected_method_ = index;
  RCLCPP_INFO(node_->get_logger(), "selected_method: %s", method_names_[index].c_str());
  
  if (index == 3 || index == 4)
  {
    if (!checkforRobotModel())
    {
      RCLCPP_ERROR(node_->get_logger(), 
        "The base placement method you selected needs robot model to be loaded.");
    }
  }
}

void PlaceBase::getSelectedOpType(int op_index)
{
  selected_op_type_ = op_index;
  RCLCPP_INFO(node_->get_logger(), "selected visualization method: %s", 
    output_type_[op_index].c_str());
  
  if (selected_op_type_ == 1 || selected_op_type_ == 2)
  {
    if (!checkforRobotModel())
    {
      RCLCPP_ERROR(node_->get_logger(), 
        "The visualization method you selected needs robot model to be loaded.");
    }
  }
}

bool PlaceBase::checkforRobotModel()
{
  if (joint_names_.empty())
  {
    if (loadRobotModel())
    {
      group_names_.clear();
      group_names_.push_back("manipulator");
      group_names_.push_back("arm");
      
      Q_EMIT sendGroupType_signal(group_names_);
      RCLCPP_INFO(node_->get_logger(), "Please select your manipulator group.");
      return true;
    }
  }
  else
  {
    return true;
  }
  return false;
}

bool PlaceBase::loadRobotModel()
{
  // TODO: Implement robot model loading from robot_description parameter
  RCLCPP_WARN(node_->get_logger(), 
    "loadRobotModel() is a stub - implement robot description parsing");
  
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  link_names_ = {"base_link", "link1", "link2", "link3", "link4", "link5", "link6"};
  
  return true;
}

void PlaceBase::getBasePoses(std::vector<geometry_msgs::msg::Pose> base_poses)
{
  final_base_poses_user = base_poses;
}

void PlaceBase::setBasePlaceParams(int base_loc_size, int high_score_sp)
{
  RCLCPP_INFO(node_->get_logger(), 
    "Base Placement parameters from UI:\n Number of base locations: %d\n Number of HighScore Spheres: %d",
    base_loc_size, high_score_sp);

  BASE_LOC_SIZE_ = base_loc_size;
  HIGH_SCORE_SP_ = high_score_sp;
  
  if (BASE_LOC_SIZE_ <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), 
      "Please provide a valid number of how many base locations do you need.");
  }

  if (HIGH_SCORE_SP_ <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), 
      "Please provide a valid number of how many spheres do you need to create valid base poses");
  }
}

void PlaceBase::createSpheres(std::multimap<std::vector<double>, std::vector<double>> basePoses,
                              std::map<std::vector<double>, double>& spColor, 
                              std::vector<std::vector<double>>& highScoredSp, 
                              bool reduce_D)
{
//   kinematics::Kinematics k;
  std::vector<int> poseCount;
  poseCount.reserve(basePoses.size());
  
  for (std::multimap<std::vector<double>, std::vector<double>>::iterator it = basePoses.begin(); 
       it != basePoses.end(); ++it)
  {
    int num = basePoses.count(it->first);
    poseCount.push_back(num);
  }
  
  std::vector<int>::const_iterator it;
  it = max_element(poseCount.begin(), poseCount.end());
  int max_number = *it;
  it = min_element(poseCount.begin(), poseCount.end());
  int min_number = *it;
  
  if (reduce_D)
  {
    for (std::multimap<std::vector<double>, std::vector<double>>::iterator it = basePoses.begin(); 
         it != basePoses.end(); ++it)
    {
      if ((it->first)[2] < 0.051 && (it->first)[2] > -0.051)
      {
        geometry_msgs::msg::Pose prob_base_pose;
        prob_base_pose.position.x = (it->first)[0];
        prob_base_pose.position.y = (it->first)[1];
        prob_base_pose.position.z = (it->first)[2];
        prob_base_pose.orientation.w = 1.0;
        
        geometry_msgs::msg::Pose base_pose_at_arm;
        transformFromRobotbaseToArmBase(prob_base_pose, base_pose_at_arm);
        
        int num_of_solns = 0;
        for (size_t j = 0; j < GRASP_POSES_.size(); j++)
        {
          std::vector<double> joint_soln;
          int nsolns = 0;
          isIkSuccesswithTransformedBase(base_pose_at_arm, GRASP_POSES_[j], joint_soln, nsolns);
          num_of_solns += nsolns;
        }

        float d = (float(num_of_solns) / float(GRASP_POSES_.size() * 8)) * 100;
        spColor.insert(std::pair<std::vector<double>, double>(it->first, double(d)));
      }
    }
  }
  else
  {
    for (std::multimap<std::vector<double>, std::vector<double>>::iterator it = basePoses.begin(); 
         it != basePoses.end(); ++it)
    {
      float d = ((float(basePoses.count(it->first)) - min_number) / (max_number - min_number)) * 100;
      if (d > 1)
      {
        spColor.insert(std::pair<std::vector<double>, double>(it->first, double(d)));
      }
    }
  }
  
  std::multiset<std::pair<double, std::vector<double>>> scoreWithSp;
  for (std::map<std::vector<double>, double>::iterator it = spColor.begin(); 
       it != spColor.end(); ++it)
  {
    scoreWithSp.insert(std::pair<double, std::vector<double>>(it->second, it->first));
  }
  
  for (std::multiset<std::pair<double, std::vector<double>>>::reverse_iterator it = scoreWithSp.rbegin();
       it != scoreWithSp.rend(); ++it)
  {
    highScoredSp.push_back(it->second);
  }
}

double PlaceBase::calculateScoreForRobotBase(std::vector<geometry_msgs::msg::Pose>& grasp_poses, 
                                             std::vector<geometry_msgs::msg::Pose>& base_poses)
{
//   kinematics::Kinematics k;
  float total_score = 0;
  float max_score = 0;
  geometry_msgs::msg::Pose best_pose;
  
  for (size_t i = 0; i < base_poses.size(); i++)
  {
    geometry_msgs::msg::Pose base_pose_at_arm;
    transformFromRobotbaseToArmBase(base_poses[i], base_pose_at_arm);
    
    int num_of_solns = 0;
    for (size_t j = 0; j < grasp_poses.size(); j++)
    {
      std::vector<double> joint_soln;
      int nsolns = 0;
      isIkSuccesswithTransformedBase(base_pose_at_arm, grasp_poses[j], joint_soln, nsolns);
      num_of_solns += nsolns;
    }
    
    float d = (float(num_of_solns) / float(grasp_poses.size() * 8)) * 100;
    if (d > max_score)
    {
      max_score = d;
      best_pose = base_poses[i];
    }
    total_score += d;
  }
  
  best_pose_ = best_pose;
  double score = double(total_score / float(base_poses.size()));
  return score;
}

double PlaceBase::calculateScoreForArmBase(std::vector<geometry_msgs::msg::Pose>& grasp_poses, 
                                           std::vector<geometry_msgs::msg::Pose>& base_poses)
{
//   kinematics::Kinematics k;
  float total_score = 0;
  float max_score = 0;
  geometry_msgs::msg::Pose best_pose;
  
  for (size_t i = 0; i < base_poses.size(); i++)
  {
    int num_of_solns = 0;
    for (size_t j = 0; j < grasp_poses.size(); j++)
    {
      std::vector<double> joint_soln;
      int nsolns = 0;
      isIkSuccesswithTransformedBase(base_poses[i], grasp_poses[j], joint_soln, nsolns);
      num_of_solns += nsolns;
    }
    
    float d = (float(num_of_solns) / float(grasp_poses.size() * 8)) * 100;
    if (d > max_score)
    {
      max_score = d;
      best_pose = base_poses[i];
    }
    total_score += d;
  }
  
  best_pose_ = best_pose;
  double score = double(total_score / float(base_poses.size()));
  return score;
}

void PlaceBase::transformFromRobotbaseToArmBase(const geometry_msgs::msg::Pose& base_pose, 
                                                geometry_msgs::msg::Pose& arm_base_pose)
{
  // TODO: Implement with proper robot model
  RCLCPP_WARN_ONCE(node_->get_logger(), 
    "transformFromRobotbaseToArmBase() using identity transform - implement with robot model");
  
  Eigen::Affine3d trans_to_arm_parent = Eigen::Affine3d::Identity();
  geometry_msgs::msg::Transform base_transform;
  base_transform.translation.x =base_pose.position.x;
  base_transform.translation.y =base_pose.position.y;
  base_transform.translation.z =base_pose.position.z;
  base_transform.rotation = base_pose.orientation;
  Eigen::Affine3d base_pose_tf = tf2::transformToEigen(base_transform);
  arm_base_pose = tf2::toMsg(base_pose_tf * trans_to_arm_parent);
}

void PlaceBase::transformToRobotbase(std::multimap<std::vector<double>, std::vector<double>> armBasePoses,
                                     std::multimap<std::vector<double>, std::vector<double>>& robotBasePoses)
{
  RCLCPP_WARN_ONCE(node_->get_logger(), 
    "transformToRobotbase() using identity transform - implement with robot model");
  
  Eigen::Affine3d arm_to_root = Eigen::Affine3d::Identity();
  
  sphere_discretization::SphereDiscretization sd;
  for (std::multimap<std::vector<double>, std::vector<double>>::iterator it = armBasePoses.begin(); 
       it != armBasePoses.end(); ++it)
  {
    geometry_msgs::msg::Pose arm_base_pose;
    sd.convertVectorToPose(it->second, arm_base_pose);
    geometry_msgs::msg::Transform arm_transform;
    arm_transform.translation.x =arm_base_pose.position.x;
    arm_transform.translation.y =arm_base_pose.position.y;
    arm_transform.translation.z =arm_base_pose.position.z;
    arm_transform.rotation = arm_base_pose.orientation;
    Eigen::Affine3d arm_base_tf = tf2::transformToEigen(arm_transform);
    geometry_msgs::msg::Pose robot_base_pose = tf2::toMsg(arm_to_root * arm_base_tf);
    
    static const int arr[] = {1, 1, 1};
    std::vector<double> base_vec(arr, arr + sizeof(arr) / sizeof(arr[0]));
    std::vector<double> base_pose;
    sd.convertPoseToVector(robot_base_pose, base_pose);
    robotBasePoses.insert(std::pair<std::vector<double>, std::vector<double>>(base_vec, base_pose));
  }
}

bool PlaceBase::findbase(std::vector<geometry_msgs::msg::Pose> grasp_poses)
{
  Q_EMIT basePlacementProcessStarted();
  score_ = 0;
  
  if (grasp_poses.size() == 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Please provide at least one grasp pose.");
    Q_EMIT basePlacementProcessFinished();
    return false;
  }
  else if (selected_method_ == 4)
  {
    GRASP_POSES_ = grasp_poses;
    BasePlaceMethodHandler();
    
    for (size_t i = 0; i < final_base_poses.size(); ++i)
    {
      tf2::Quaternion quat(final_base_poses[i].orientation.x, final_base_poses[i].orientation.y,
                           final_base_poses[i].orientation.z, final_base_poses[i].orientation.w);
      tf2::Matrix3x3 m(quat);

      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      RCLCPP_INFO(node_->get_logger(), 
        "Optimal base pose[%zu]: Position: %f, %f, %f, Orientation: %f, %f, %f", 
        i + 1, final_base_poses[i].position.x, final_base_poses[i].position.y, 
        final_base_poses[i].position.z, roll, pitch, yaw);
    }
    OuputputVizHandler(final_base_poses);
  }
  else
  {
    if (PoseColFilter.size() == 0)
    {
      RCLCPP_INFO(node_->get_logger(), 
        "No Inverse Reachability Map found. Please provide an Inverse Reachability map.");
    }
    else
    {
      sphere_discretization::SphereDiscretization sd;
      baseTrnsCol.clear();
      sphereColor.clear();
      highScoreSp.clear();
      robot_PoseColfilter.clear();
      GRASP_POSES_.clear();
      final_base_poses.clear();
      GRASP_POSES_ = grasp_poses;

      if (selected_method_ == 3)
      {
        transformToRobotbase(PoseColFilter, robot_PoseColfilter);
        sd.associatePose(baseTrnsCol, grasp_poses, robot_PoseColfilter, res);
        RCLCPP_INFO(node_->get_logger(), "Size of baseTrnsCol dataset: %lu", baseTrnsCol.size());
        createSpheres(baseTrnsCol, sphereColor, highScoreSp, true);
      }
      else
      {
        sd.associatePose(baseTrnsCol, grasp_poses, PoseColFilter, res);
        RCLCPP_INFO(node_->get_logger(), "Size of baseTrnsCol dataset: %lu", baseTrnsCol.size());
        createSpheres(baseTrnsCol, sphereColor, highScoreSp, false);
      }

      RCLCPP_INFO(node_->get_logger(), "Union map has been created. Can now visualize Union Map.");
      RCLCPP_INFO(node_->get_logger(), "Poses in Union Map: %lu", baseTrnsCol.size());
      RCLCPP_INFO(node_->get_logger(), "Spheres in Union Map: %lu", sphereColor.size());

      BasePlaceMethodHandler();

      for (size_t i = 0; i < final_base_poses.size(); ++i)
      {
        tf2::Quaternion quat(final_base_poses[i].orientation.x, final_base_poses[i].orientation.y,
                             final_base_poses[i].orientation.z, final_base_poses[i].orientation.w);
        tf2::Matrix3x3 m(quat);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        RCLCPP_INFO(node_->get_logger(), 
          "Optimal base pose[%zu]: Position: %f, %f, %f, Orientation: %f, %f, %f", 
          i + 1, final_base_poses[i].position.x, final_base_poses[i].position.y, 
          final_base_poses[i].position.z, roll, pitch, yaw);
      }

      OuputputVizHandler(final_base_poses);
    }
  }

  tf2::Quaternion quat(best_pose_.orientation.x, best_pose_.orientation.y, 
                       best_pose_.orientation.z, best_pose_.orientation.w);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  RCLCPP_INFO(node_->get_logger(), 
    "Best pose for this solution: Position: %f, %f, %f, Orientation: %f, %f, %f",
    best_pose_.position.x, best_pose_.position.y, best_pose_.position.z, roll, pitch, yaw);

  Q_EMIT basePlacementProcessCompleted(score_);
  Q_EMIT basePlacementProcessFinished();
  RCLCPP_INFO(node_->get_logger(), "FindBase Task Finished");
  return true;
}

void PlaceBase::BasePlaceMethodHandler()
{
  switch (selected_method_)
  {
    case 0:
      findBaseByPCA();
      break;
    case 1:
      findBaseByGraspReachabilityScore();
      break;
    case 2:
      findBaseByIKSolutionScore();
      break;
    case 3:
      findBaseByVerticalRobotModel();
      break;
    case 4:
      findBaseByUserIntuition();
      break;
  }
}

void PlaceBase::OuputputVizHandler(std::vector<geometry_msgs::msg::Pose> po)
{
  switch (selected_op_type_)
  {
    case 0:
      showBaseLocationsbyArrow(po);
      break;
    case 1:
      showBaseLocationsbyArmModel(po);
      break;
    case 2:
      showBaseLocationsbyRobotModel(po);
      break;
  }
}

void PlaceBase::findBaseByUserIntuition()
{
  RCLCPP_INFO(node_->get_logger(), "Finding optimal base pose by user intuition.");
  double s = calculateScoreForRobotBase(GRASP_POSES_, final_base_poses_user);
  score_ = s;
  final_base_poses = final_base_poses_user;
}

void PlaceBase::findBaseByVerticalRobotModel()
{
  RCLCPP_INFO(node_->get_logger(), "Finding optimal ROBOT base pose by Vertical robot model.");
  std::vector<geometry_msgs::msg::Pose> base_poses;
  std::vector<geometry_msgs::msg::Pose> base_poses_user;
  
  int num_of_desired_sp = HIGH_SCORE_SP_ + 50;
  int num_of_sp = highScoreSp.size();
  
  if (BASE_LOC_SIZE_ > num_of_sp)
  {
    RCLCPP_ERROR(node_->get_logger(), "Desired base locations are too high. Please reduce it");
  }
  
  for (int i = 0; i < num_of_desired_sp; i += 4)
  {
    geometry_msgs::msg::Pose prob_base_pose;
    prob_base_pose.position.x = highScoreSp[i][0];
    prob_base_pose.position.y = highScoreSp[i][1];
    prob_base_pose.position.z = highScoreSp[i][2];
    prob_base_pose.orientation.x = 0;
    prob_base_pose.orientation.y = 0;
    prob_base_pose.orientation.z = 0;
    prob_base_pose.orientation.w = 1;
    base_poses.push_back(prob_base_pose);
  }

  for (int i = 0; i < BASE_LOC_SIZE_; ++i)
  {
    base_poses_user.push_back(base_poses[i]);
  }

  double s = calculateScoreForRobotBase(GRASP_POSES_, base_poses_user);
  score_ = s;
  final_base_poses = base_poses_user;
}

void PlaceBase::findBaseByPCA()
{
  RCLCPP_INFO(node_->get_logger(), "Finding optimal base pose by PCA.");
  sphere_discretization::SphereDiscretization sd;
  std::vector<geometry_msgs::msg::Pose> pose_scores;
  PlaceBase::WorkSpace ws;
  
  for (int i = 0; i < BASE_LOC_SIZE_; ++i)
  {
    PlaceBase::WsSphere wss;
    wss.point.x = highScoreSp[i][0];
    wss.point.y = highScoreSp[i][1];
    wss.point.z = highScoreSp[i][2];
    
    std::vector<double> basePose;
    basePose.push_back(highScoreSp[i][0]);
    basePose.push_back(highScoreSp[i][1]);
    basePose.push_back(highScoreSp[i][2]);
    
    std::multimap<std::vector<double>, std::vector<double>>::iterator it;
    for (it = baseTrnsCol.lower_bound(basePose); it != baseTrnsCol.upper_bound(basePose); ++it)
    {
      geometry_msgs::msg::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      wss.poses.push_back(pp);
    }
    ws.ws_spheres.push_back(wss);
  }

  for (size_t i = 0; i < ws.ws_spheres.size(); ++i)
  {
    geometry_msgs::msg::Pose final_base_pose;
    sd.findOptimalPosebyPCA(ws.ws_spheres[i].poses, final_base_pose);
    
    final_base_pose.position.x = ws.ws_spheres[i].point.x;
    final_base_pose.position.y = ws.ws_spheres[i].point.y;
    final_base_pose.position.z = ws.ws_spheres[i].point.z;
    geometry_msgs::msg::Transform final_transform;
    final_transform.translation.x =final_base_pose.position.x;
    final_transform.translation.y =final_base_pose.position.y;
    final_transform.translation.z =final_base_pose.position.z;
    final_transform.rotation = final_base_pose.orientation;
    Eigen::Affine3d final_base_tf = tf2::transformToEigen(final_transform);
    Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d(1, 0, 0)));
    geometry_msgs::msg::Pose new_base_pose = tf2::toMsg(final_base_tf * rx);

    pose_scores.push_back(new_base_pose);
  }

  double s = calculateScoreForArmBase(GRASP_POSES_, pose_scores);
  score_ = s;
  final_base_poses = pose_scores;
}

void PlaceBase::findBaseByGraspReachabilityScore()
{
  RCLCPP_INFO(node_->get_logger(), "Finding optimal base pose by GraspReachabilityScore.");
  sphere_discretization::SphereDiscretization sd;
//   kinematics::Kinematics kin;
  std::vector<geometry_msgs::msg::Pose> pose_scores;
  int numofSp = BASE_LOC_SIZE_;

  for (int i = 0; i < numofSp; i++)
  {
    std::vector<geometry_msgs::msg::Pose> probBasePoses;
    std::multimap<std::vector<double>, std::vector<double>>::iterator it;
    
    for (it = baseTrnsCol.lower_bound(highScoreSp[i]); 
         it != baseTrnsCol.upper_bound(highScoreSp[i]); ++it)
    {
      geometry_msgs::msg::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      probBasePoses.push_back(pp);
    }
    
    std::map<int, geometry_msgs::msg::Pose> basePoseWithHits;
    for (size_t j = 0; j < probBasePoses.size(); j++)
    {
      int numofHits = 0;
      for (size_t j1 = 0; j1 < GRASP_POSES_.size(); j1++)
      {
        int nsolns = 0;
        std::vector<double> joint_solns;
        numofHits += isIkSuccesswithTransformedBase(probBasePoses[j], GRASP_POSES_[j1], 
                                                        joint_solns, nsolns);
      }
      basePoseWithHits.insert(std::make_pair(numofHits, probBasePoses[j]));
    }
    
    std::map<int, geometry_msgs::msg::Pose>::iterator itr;
    itr = basePoseWithHits.end();
    --itr;
    pose_scores.push_back(itr->second);
  }
  
  double s = calculateScoreForArmBase(GRASP_POSES_, pose_scores);
  score_ = s;
  final_base_poses = pose_scores;
}

void PlaceBase::findBaseByIKSolutionScore()
{
  RCLCPP_INFO(node_->get_logger(), "Finding optimal base pose by IKSolutionScore.");
  sphere_discretization::SphereDiscretization sd;
//   kinematics::Kinematics kin;
  std::vector<geometry_msgs::msg::Pose> pose_scores;
  int numofSp = BASE_LOC_SIZE_;

  int max_solns = GRASP_POSES_.size() * 8;
  int min_solns = 0;

  for (int i = 0; i < numofSp; i++)
  {
    std::vector<geometry_msgs::msg::Pose> probBasePoses;
    std::multimap<std::vector<double>, std::vector<double>>::iterator it;
    
    for (it = baseTrnsCol.lower_bound(highScoreSp[i]); 
         it != baseTrnsCol.upper_bound(highScoreSp[i]); ++it)
    {
      geometry_msgs::msg::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      probBasePoses.push_back(pp);
    }
    
    std::map<double, geometry_msgs::msg::Pose> basePoseWithHits;
    for (size_t j = 0; j < probBasePoses.size(); j++)
    {
      int numofHits = 0;
      int solns = 0;
      for (size_t j1 = 0; j1 < GRASP_POSES_.size(); j1++)
      {
        int nsolns = 0;
        std::vector<double> joint_solns;
        numofHits += isIkSuccesswithTransformedBase(probBasePoses[j], GRASP_POSES_[j1], 
                                                        joint_solns, nsolns);
        solns += nsolns;
      }
      double basePlaceScore = (double(solns) - double(min_solns)) / 
                              (double(max_solns) - double(min_solns));
      basePoseWithHits.insert(std::make_pair(basePlaceScore, probBasePoses[j]));
    }
    
    std::map<double, geometry_msgs::msg::Pose>::iterator itr;
    itr = basePoseWithHits.end();
    --itr;
    pose_scores.push_back(itr->second);
  }
  
  double s = calculateScoreForArmBase(GRASP_POSES_, pose_scores);
  score_ = s;
  final_base_poses = pose_scores;
}

void PlaceBase::showBaseLocationsbyArrow(std::vector<geometry_msgs::msg::Pose> po)
{
  RCLCPP_INFO(node_->get_logger(), 
    "Showing Base Locations by Arrow: Arrows are pointing in Z direction");
  
  std::vector<geometry_msgs::msg::Pose> pose_arr;
  for (size_t i = 0; i < po.size(); i++)
  {
    tf2::Transform trns;
    tf2::Quaternion quat(po[i].orientation.x, po[i].orientation.y, 
                         po[i].orientation.z, po[i].orientation.w);
    tf2::Vector3 vec(po[i].position.x, po[i].position.y, po[i].position.z);
    trns.setOrigin(vec);
    trns.setRotation(quat);

    tf2::Quaternion quat2;
    quat2.setRPY(0, -M_PI / 2, 0);
    tf2::Vector3 vec2(0, 0, 0);
    tf2::Transform multiplier;
    multiplier.setOrigin(vec2);
    multiplier.setRotation(quat2);

    trns = trns * multiplier;
    tf2::Vector3 new_pose_vec = trns.getOrigin();
    tf2::Quaternion new_pose_quat = trns.getRotation();
    new_pose_quat.normalize();

    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = new_pose_vec[0];
    new_pose.position.y = new_pose_vec[1];
    new_pose.position.z = new_pose_vec[2];
    new_pose.orientation.x = new_pose_quat[0];
    new_pose.orientation.y = new_pose_quat[1];
    new_pose.orientation.z = new_pose_quat[2];
    new_pose.orientation.w = new_pose_quat[3];
    pose_arr.push_back(new_pose);
  }

  auto marker_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "visualization_marker_array", 1);
  
  visualization_msgs::msg::MarkerArray markerArr;
  for (size_t i = 0; i < po.size(); ++i)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = node_->now();
    marker.ns = "points";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.scale.x = 0.3;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.id = static_cast<int>(i);

    marker.pose = pose_arr[i];
    markerArr.markers.push_back(marker);
  }
  marker_pub->publish(markerArr);
}

void PlaceBase::showBaseLocationsbyArmModel(std::vector<geometry_msgs::msg::Pose> po)
{
  RCLCPP_INFO(node_->get_logger(), "Showing Base Locations by Arm Model:");
//   kinematics::Kinematics k;

  auto imServer = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "robot_model", node_);

  std::vector<visualization_msgs::msg::InteractiveMarker> iMarkers;
  BasePoseJoint base_pose_joints;
  
  for (size_t i = 0; i < po.size(); i++)
  {
    for (size_t j = 0; j < GRASP_POSES_.size(); j++)
    {
      std::vector<double> joint_soln;
      int nsolns = 0;
      isIkSuccesswithTransformedBase(po[i], GRASP_POSES_[j], joint_soln, nsolns);
      std::pair<std::vector<double>, geometry_msgs::msg::Pose> myPair(joint_soln, po[i]);
      base_pose_joints.insert(myPair);
    }
  }
  
  mark_->makeArmMarker(base_pose_joints, iMarkers, show_ureach_models_);
  
  for (size_t i = 0; i < iMarkers.size(); i++)
  {
    imServer->insert(iMarkers[i]);
  }

  imServer->applyChanges();
  rclcpp::sleep_for(std::chrono::seconds(25));
}

void PlaceBase::showBaseLocationsbyRobotModel(std::vector<geometry_msgs::msg::Pose> po)
{
  RCLCPP_INFO(node_->get_logger(), "Showing Base Locations by Robot Model:");
//   kinematics::Kinematics k;
  
  auto imServer = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "robot_model", node_);

  std::vector<visualization_msgs::msg::InteractiveMarker> iMarkers;
  BasePoseJoint base_pose_joints;
  
  for (size_t i = 0; i < po.size(); i++)
  {
    geometry_msgs::msg::Pose base_pose_at_arm;
    transformFromRobotbaseToArmBase(po[i], base_pose_at_arm);
    
    for (size_t j = 0; j < GRASP_POSES_.size(); j++)
    {
      std::vector<double> joint_soln;
      int nsolns = 0;
      isIkSuccesswithTransformedBase(base_pose_at_arm, GRASP_POSES_[j], joint_soln, nsolns);
      std::pair<std::vector<double>, geometry_msgs::msg::Pose> myPair(joint_soln, po[i]);
      base_pose_joints.insert(myPair);
    }
  }

  mark_->makeRobotMarker(base_pose_joints, iMarkers, show_ureach_models_);
  
  for (size_t i = 0; i < iMarkers.size(); i++)
  {
    imServer->insert(iMarkers[i]);
  }

  imServer->applyChanges();
  rclcpp::sleep_for(std::chrono::seconds(25));
}

void PlaceBase::setReachabilityData(
  std::multimap<std::vector<double>, std::vector<double>> PoseCollection,
  std::multimap<std::vector<double>, double> SphereCollection, 
  float resolution)
{
  PoseColFilter = PoseCollection;
  SphereCol = SphereCollection;
  res = resolution;
  
  RCLCPP_INFO(node_->get_logger(), "Size of poses dataset: %lu", PoseColFilter.size());
  RCLCPP_INFO(node_->get_logger(), "Size of Sphere dataset: %lu", SphereCol.size());
  RCLCPP_INFO(node_->get_logger(), "Resolution: %f", res);
}

void PlaceBase::ShowUnionMap(bool /* show_map */)
{
  RCLCPP_INFO(node_->get_logger(), "Showing Map:");
  
  if (sphereColor.size() == 0)
  {
    RCLCPP_INFO(node_->get_logger(), 
      "The union map has not been created yet. Please create the Union map by the Find Base button");
  }
  else
  {
    // change it for a point cloud
    // auto workspace_pub = node_->create_publisher<PlaceBase::WorkSpace>("reachability_map", 1);
    
    PlaceBase::WorkSpace ws;
    ws.header.stamp = node_->now();
    ws.header.frame_id = "base_link";
    ws.resolution = res;

    for (std::multimap<std::vector<double>, double>::iterator it = sphereColor.begin(); 
         it != sphereColor.end(); ++it)
    {
      PlaceBase::WsSphere wss;
      wss.point.x = it->first[0];
      wss.point.y = it->first[1];
      wss.point.z = it->first[2];
      wss.ri = (float)it->second;
      ws.ws_spheres.push_back(wss);
    }
    // workspace_pub->publish(ws);
  }
}

void PlaceBase::clearUnionMap(bool /* clear_map */)
{
  RCLCPP_INFO(node_->get_logger(), "Clearing Map:");
  // TODO: Implement clearing functionality
}