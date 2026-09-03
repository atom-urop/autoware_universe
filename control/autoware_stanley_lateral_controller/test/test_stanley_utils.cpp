#include "autoware/stanley_lateral_controller/stanley_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::motion::control::stanley_lateral_controller
{

TEST(StanleyUtilsTest, RearToFrontOdometryStraightMotion)
{
  nav_msgs::msg::Odometry rear_pose;

  // Rear axle position
  rear_pose.pose.pose.position.x = 0.0;
  rear_pose.pose.pose.position.y = 0.0;

  // Yaw = 0
  rear_pose.pose.pose.orientation.w = 1.0;
  rear_pose.pose.pose.orientation.x = 0.0;
  rear_pose.pose.pose.orientation.y = 0.0;
  rear_pose.pose.pose.orientation.z = 0.0;

  // Rear axle velocity
  rear_pose.twist.twist.linear.x = 5.0;
  rear_pose.twist.twist.linear.y = 0.0;
  rear_pose.twist.twist.angular.z = 0.0;

  const double wheel_base = 2.0;

  const auto front_pose =
    rearToFrontOdometry(rear_pose, wheel_base);

  EXPECT_NEAR(
    front_pose.pose.pose.position.x,
    2.0,
    1e-9);

  EXPECT_NEAR(
    front_pose.pose.pose.position.y,
    0.0,
    1e-9);

  EXPECT_NEAR(
    front_pose.twist.twist.linear.x,
    5.0,
    1e-9);

  EXPECT_NEAR(
    front_pose.twist.twist.linear.y,
    0.0,
    1e-9);
}

TEST(StanleyUtilsTest, RearToFrontOdometryPredStraightMotion)
{
  nav_msgs::msg::Odometry rear_pose;

  // Rear axle position
  rear_pose.pose.pose.position.x = 0.0;
  rear_pose.pose.pose.position.y = 0.0;

  // Yaw = 0
  rear_pose.pose.pose.orientation.w = 1.0;
  rear_pose.pose.pose.orientation.x = 0.0;
  rear_pose.pose.pose.orientation.y = 0.0;
  rear_pose.pose.pose.orientation.z = 0.0;

  // Rear axle velocity
  rear_pose.twist.twist.linear.x = 5.0;
  rear_pose.twist.twist.linear.y = 0.0;
  rear_pose.twist.twist.angular.z = 0.0;

  const double wheel_base = 2.0;
  const double tau_max = 0.27;
  const double d0 = 0.3;

  const auto predicted_front_pose =
    rearToFrontOdometryPred(
      rear_pose,
      wheel_base,
      tau_max,
      d0);

  // tau_pos = min(0.27, 0.3 / 5.0) = 0.06 s
  // Predicted rear position = 5.0 * 0.06 = 0.3 m
  // Predicted front position = 0.3 + 2.0 = 2.3 m

  EXPECT_NEAR(
    predicted_front_pose.pose.pose.position.x,
    2.3,
    1e-9);

  EXPECT_NEAR(
    predicted_front_pose.pose.pose.position.y,
    0.0,
    1e-9);

  EXPECT_NEAR(
    predicted_front_pose.twist.twist.linear.x,
    5.0,
    1e-9);

  EXPECT_NEAR(
    predicted_front_pose.twist.twist.linear.y,
    0.0,
    1e-9);
}

}  // namespace autoware::motion::control::stanley_lateral_controller