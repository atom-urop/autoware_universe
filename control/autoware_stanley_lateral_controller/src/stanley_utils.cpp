#include "autoware/stanley_lateral_controller/stanley_utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include "autoware_utils/math/normalization.hpp"
#include "autoware/interpolation/linear_interpolation.hpp"

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>

namespace autoware::motion::control::stanley_lateral_controller
{

double calculate4WSGain(
  const double reference_curvature,
  const std::vector<double> & kappa_gain_LUT,
  const std::vector<double> & gain_4WS_LUT)
{
  return autoware::interpolation::lerp(
    kappa_gain_LUT,
    gain_4WS_LUT,
    std::abs(reference_curvature));
}

double calculateRearSteeringRatio(
  const double reference_curvature,
  const std::vector<double> & k_ref_LUT,
  const std::vector<double> & rr_LUT)
{
  return autoware::interpolation::lerp(
    k_ref_LUT,
    rr_LUT,
    std::abs(reference_curvature));
}

std::vector<double> calcCurvatureVectorStanley(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const double traj_resample_dist,
  const double curvature_calculation_distance)
{
  const size_t n = trajectory.points.size();

  if (n < 3) {
    return std::vector<double>(n, 0.0);
  }

  // Calculate the index distance corresponding to the desired
  // curvature calculation distance.
  size_t idx_dist = static_cast<size_t>(
    std::max(
      static_cast<int>(curvature_calculation_distance / traj_resample_dist),
      1));

  // Same limitation used by Autoware.
  const auto max_idx_dist =
    static_cast<size_t>(
      std::floor(
        static_cast<double>(n - 1) / 2.0));

  idx_dist = std::max(
    static_cast<size_t>(1),
    std::min(idx_dist, max_idx_dist));

  std::vector<double> k_arr(n, 0.0);

  // Same three-point curvature calculation used by Autoware.
  for (size_t i = 1; i + 1 < n; ++i) {
    const auto & p0 =
      trajectory.points.at(
        i - std::min(idx_dist, i)).pose.position;

    const auto & p1 =
      trajectory.points.at(i).pose.position;

    const auto & p2 =
      trajectory.points.at(
        i + std::min(idx_dist, n - 1 - i)).pose.position;

    try {
      k_arr.at(i) =
        autoware_utils_geometry::calc_curvature(p0, p1, p2);
    } catch (const std::exception &) {
      if (i > 1) {
        k_arr.at(i) = k_arr.at(i - 1);
      } else {
        k_arr.at(i) = 0.0;
      }
    }
  }

  // Same endpoint handling used by Autoware.
  k_arr.at(0) = k_arr.at(1);
  k_arr.back() = k_arr.at(n - 2);

  return k_arr;
}

double calcLateralErrorStanley(
  const nav_msgs::msg::Odometry & ego_odometry,
  const geometry_msgs::msg::Pose & nearest_pose)
{
  const double err_x =
    ego_odometry.pose.pose.position.x - nearest_pose.position.x;

  const double err_y =
    ego_odometry.pose.pose.position.y - nearest_pose.position.y;

  const double ref_yaw =
    tf2::getYaw(nearest_pose.orientation);

  return
    std::sin(ref_yaw) * err_x -
    std::cos(ref_yaw) * err_y;
}

geometry_msgs::msg::Pose calcNearestPoseInterpStanley(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & self_pose,
  const double max_dist,
  const double max_yaw,
  size_t & nearest_idx)
{

  nearest_idx = 0;

  if (trajectory.points.empty()) {
    return geometry_msgs::msg::Pose{};
  }

  nearest_idx =
    autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      trajectory.points, self_pose, max_dist, max_yaw);

    const size_t traj_size = trajectory.points.size();

  if (traj_size == 1) {
    return trajectory.points.at(0).pose;
  }

  size_t prev_idx;
  size_t next_idx;

  if (nearest_idx == 0) {
    prev_idx = 0;
    next_idx = 1;
  } else if (nearest_idx == traj_size - 1) {
    prev_idx = traj_size - 2;
    next_idx = traj_size - 1;
  } else {
    const double signed_length =
      autoware::motion_utils::calcLongitudinalOffsetToSegment(
        trajectory.points, nearest_idx, self_pose.position);

    if (signed_length <= 0.0) {
      prev_idx = nearest_idx - 1;
      next_idx = nearest_idx;
    } else {
      prev_idx = nearest_idx;
      next_idx = nearest_idx + 1;
    }
  }

  const auto & prev_point = trajectory.points.at(prev_idx);
  const auto & next_point = trajectory.points.at(next_idx);

  const double segment_length =
    autoware_utils::calc_distance2d(
      prev_point.pose.position,
      next_point.pose.position);

  if (segment_length < 1.0E-5) {
    return trajectory.points.at(nearest_idx).pose;
  }

  const double longitudinal_offset =
    autoware::motion_utils::calcLongitudinalOffsetToSegment(
      trajectory.points,
      prev_idx,
      self_pose.position);

  const double ratio =
    std::clamp(longitudinal_offset / segment_length, 0.0, 1.0);

  geometry_msgs::msg::Pose nearest_pose;

  nearest_pose.position.x =
    (1.0 - ratio) * prev_point.pose.position.x +
    ratio * next_point.pose.position.x;

  nearest_pose.position.y =
    (1.0 - ratio) * prev_point.pose.position.y +
    ratio * next_point.pose.position.y;

  const double prev_yaw =
    tf2::getYaw(prev_point.pose.orientation);

  const double next_yaw =
    tf2::getYaw(next_point.pose.orientation);

  const double yaw_error =
    autoware_utils_math::normalize_radian(prev_yaw - next_yaw);

  const double nearest_yaw =
    autoware_utils_math::normalize_radian(
      next_yaw + (1.0 - ratio) * yaw_error);

  nearest_pose.orientation =
    autoware::universe_utils::createQuaternionFromYaw(nearest_yaw);

  return nearest_pose;
}

 nav_msgs::msg::Odometry rearToFrontOdometry(
  const nav_msgs::msg::Odometry & rear_pose,
  const double wheel_base)
{
  nav_msgs::msg::Odometry front_pose = rear_pose;

  // Current rear axle state
  const double x_r = rear_pose.pose.pose.position.x;
  const double y_r = rear_pose.pose.pose.position.y;
  const double yaw = tf2::getYaw(rear_pose.pose.pose.orientation);

  // Rear axle velocity in body frame
  const double vx_body = rear_pose.twist.twist.linear.x;
  const double vy_body = rear_pose.twist.twist.linear.y;
  const double yaw_rate = rear_pose.twist.twist.angular.z;

  // Rear axle -> front axle
  const double x_f =
    x_r + wheel_base * std::cos(yaw);

  const double y_f =
    y_r + wheel_base * std::sin(yaw);

  // Front axle velocity in body frame
  const double vx_front_body = vx_body;
  const double vy_front_body =
    vy_body + yaw_rate * wheel_base;

  // Front axle pose
  front_pose.pose.pose.position.x = x_f;
  front_pose.pose.pose.position.y = y_f;
  front_pose.pose.pose.position.z = rear_pose.pose.pose.position.z;

  front_pose.pose.pose.orientation =
    autoware::universe_utils::createQuaternionFromYaw(yaw);

  // Front axle velocity in body frame
  front_pose.twist.twist.linear.x = vx_front_body;
  front_pose.twist.twist.linear.y = vy_front_body;
  front_pose.twist.twist.linear.z =
    rear_pose.twist.twist.linear.z;

  return front_pose;
}

nav_msgs::msg::Odometry rearToFrontOdometryPred(
  const nav_msgs::msg::Odometry & rear_pose,
  const double wheel_base,
  const double tau_max,
  const double d0)
{
  nav_msgs::msg::Odometry front_pose = rear_pose;

  // Current rear axle state
  const double x_r = rear_pose.pose.pose.position.x;
  const double y_r = rear_pose.pose.pose.position.y;
  const double yaw = tf2::getYaw(rear_pose.pose.pose.orientation);

  // Rear axle velocity in body frame
  const double vx_body = rear_pose.twist.twist.linear.x;
  const double vy_body = rear_pose.twist.twist.linear.y;
  const double yaw_rate = rear_pose.twist.twist.angular.z;

  // Preview time
  const double vx = std::max(std::abs(vx_body), 0.1);
  const double tau_pos = std::min(tau_max, d0 / vx);

  // Rear axle velocity: body frame -> world frame
  const double vx_world =
    vx_body * std::cos(yaw) - vy_body * std::sin(yaw);

  const double vy_world =
    vx_body * std::sin(yaw) + vy_body * std::cos(yaw);

  // Predict rear axle position
  const double x_r_pred = x_r + vx_world * tau_pos;
  const double y_r_pred = y_r + vy_world * tau_pos;

  // Predict yaw
  const double yaw_pred = yaw + yaw_rate * tau_pos;

  // Rear axle -> front axle
  const double x_f_pred =
    x_r_pred + wheel_base * std::cos(yaw_pred);

  const double y_f_pred =
    y_r_pred + wheel_base * std::sin(yaw_pred);

  // Front axle pose
  front_pose.pose.pose.position.x = x_f_pred;
  front_pose.pose.pose.position.y = y_f_pred;
  front_pose.pose.pose.position.z = rear_pose.pose.pose.position.z;

  front_pose.pose.pose.orientation =
    autoware::universe_utils::createQuaternionFromYaw(yaw_pred);

  // Front axle velocity in body frame
  front_pose.twist.twist.linear.x = vx_body;
  front_pose.twist.twist.linear.y =
    vy_body + yaw_rate * wheel_base;
  front_pose.twist.twist.linear.z =
    rear_pose.twist.twist.linear.z;

  return front_pose;
}

}  // namespace autoware::motion::control::stanley_lateral_controller