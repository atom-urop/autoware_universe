#ifndef AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_UTILS_HPP_
#define AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_UTILS_HPP_

#include "nav_msgs/msg/odometry.hpp"

namespace autoware::motion::control::stanley_lateral_controller
{

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
