#include "autoware/stanley_lateral_controller/stanley_utils.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>

namespace autoware::motion::control::stanley_lateral_controller
{

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