#ifndef AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_UTILS_HPP_
#define AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_UTILS_HPP_

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"

namespace autoware::motion::control::stanley_lateral_controller
{

double calculate4WSGain(
  double reference_curvature,
  const std::vector<double> & kappa_gain_LUT,
  const std::vector<double> & gain_4WS_LUT);

double calculateRearSteeringRatio(
  double reference_curvature,
  const std::vector<double> & k_ref_LUT,
  const std::vector<double> & rr_LUT);

std::vector<double> calcCurvatureVectorStanley(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const double traj_resample_dist,
  const double curvature_calculation_distance);

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
