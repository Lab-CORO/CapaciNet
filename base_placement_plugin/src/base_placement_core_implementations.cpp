// This file contains the ported algorithm implementations
// To integrate: Copy these implementations to base_placement_core.cpp

#include <base_placement_plugin/base_placement_core.h>
#include <base_placement_plugin/sphere_discretization.h>
#include <Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.hpp>

// ============================================================
// HELPER METHODS - Transform Functions
// ============================================================

void BasePlacementCore::transformToRobotbase(
  std::multimap<std::vector<double>, std::vector<double>> armBasePoses,
  std::multimap<std::vector<double>, std::vector<double>>& robotBasePoses)
{
  RCLCPP_WARN_ONCE(node_->get_logger(),
    "transformToRobotbase() using identity transform - implement with robot model");

  Eigen::Affine3d arm_to_root = Eigen::Affine3d::Identity();

  sphere_discretization::SphereDiscretization sd;
  for (auto it = armBasePoses.begin(); it != armBasePoses.end(); ++it)
  {
    geometry_msgs::msg::Pose arm_base_pose;
    sd.convertVectorToPose(it->second, arm_base_pose);

    geometry_msgs::msg::Transform arm_transform;
    arm_transform.translation.x = arm_base_pose.position.x;
    arm_transform.translation.y = arm_base_pose.position.y;
    arm_transform.translation.z = arm_base_pose.position.z;
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

void BasePlacementCore::transformFromRobotbaseToArmBase(
  const geometry_msgs::msg::Pose& base_pose,
  geometry_msgs::msg::Pose& arm_base_pose)
{
  RCLCPP_WARN_ONCE(node_->get_logger(),
    "transformFromRobotbaseToArmBase() using identity transform - implement with robot model");

  Eigen::Affine3d trans_to_arm_parent = Eigen::Affine3d::Identity();

  geometry_msgs::msg::Transform base_transform;
  base_transform.translation.x = base_pose.position.x;
  base_transform.translation.y = base_pose.position.y;
  base_transform.translation.z = base_pose.position.z;
  base_transform.rotation = base_pose.orientation;

  Eigen::Affine3d base_pose_tf = tf2::transformToEigen(base_transform);
  arm_base_pose = tf2::toMsg(base_pose_tf * trans_to_arm_parent);
}

// ============================================================
// HELPER METHODS - Sphere Creation & Scoring
// ============================================================

void BasePlacementCore::createSpheres(
  std::multimap<std::vector<double>, std::vector<double>> basePoses,
  std::map<std::vector<double>, double>& spColor,
  std::vector<std::vector<double>>& highScoredSp,
  bool reduce_D)
{
  std::vector<int> poseCount;
  poseCount.reserve(basePoses.size());

  for (auto it = basePoses.begin(); it != basePoses.end(); ++it)
  {
    int num = basePoses.count(it->first);
    poseCount.push_back(num);
  }

  auto max_it = std::max_element(poseCount.begin(), poseCount.end());
  auto min_it = std::min_element(poseCount.begin(), poseCount.end());
  int max_number = *max_it;
  int min_number = *min_it;

  if (reduce_D)
  {
    // For vertical robot model - check Z coordinate
    std::vector<geometry_msgs::msg::Pose> task_poses_copy;
    for (const auto& [name, pose] : named_task_poses_)
    {
      task_poses_copy.push_back(pose);
    }

    for (auto it = basePoses.begin(); it != basePoses.end(); ++it)
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
        for (const auto& grasp_pose : task_poses_copy)
        {
          std::vector<double> joint_soln;
          int nsolns = 0;
          isIkSuccesswithTransformedBase(base_pose_at_arm, grasp_pose, joint_soln, nsolns);
          num_of_solns += nsolns;
        }

        float d = (float(num_of_solns) / float(task_poses_copy.size() * 8)) * 100;
        spColor.insert(std::pair<std::vector<double>, double>(it->first, double(d)));
      }
    }
  }
  else
  {
    for (auto it = basePoses.begin(); it != basePoses.end(); ++it)
    {
      float d = ((float(basePoses.count(it->first)) - min_number) / (max_number - min_number)) * 100;
      if (d > 1)
      {
        spColor.insert(std::pair<std::vector<double>, double>(it->first, double(d)));
      }
    }
  }

  // Sort by score
  std::multiset<std::pair<double, std::vector<double>>> scoreWithSp;
  for (auto it = spColor.begin(); it != spColor.end(); ++it)
  {
    scoreWithSp.insert(std::pair<double, std::vector<double>>(it->second, it->first));
  }

  for (auto it = scoreWithSp.rbegin(); it != scoreWithSp.rend(); ++it)
  {
    highScoredSp.push_back(it->second);
  }
}

double BasePlacementCore::calculateScoreForRobotBase(
  std::vector<geometry_msgs::msg::Pose>& grasp_poses,
  std::vector<geometry_msgs::msg::Pose>& base_poses)
{
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

double BasePlacementCore::calculateScoreForArmBase(
  std::vector<geometry_msgs::msg::Pose>& grasp_poses,
  std::vector<geometry_msgs::msg::Pose>& base_poses)
{
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
