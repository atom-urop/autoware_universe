#ifndef AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_UTILS_HPP_
#define AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_UTILS_HPP_

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"

namespace autoware::motion::control::stanley_lateral_controller
{

double calcReferenceCurvatureStanley(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  size_t nearest_idx,
  double traj_resample_dist,
  double curvature_calculation_distance);

double calcLateralErrorStanley(
  const nav_msgs::msg::Odometry & ego_odometry,
  const geometry_msgs::msg::Pose & nearest_pose);

geometry_msgs::msg::Pose calcNearestPoseInterpStanley(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & self_pose,
  double max_dist,
  double max_yaw,
  size_t & nearest_idx);

nav_msgs::msg::Odometry rearToFrontOdometry(
  const nav_msgs::msg::Odometry & rear_pose,
  double wheel_base);

nav_msgs::msg::Odometry rearToFrontOdometryPred(
  const nav_msgs::msg::Odometry & rear_pose,
  double wheel_base,
  double tau_max,
  double d0);

}  // namespace autoware::motion::control::stanley_lateral_controller

#endif  // AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_UTILS_HPP_
