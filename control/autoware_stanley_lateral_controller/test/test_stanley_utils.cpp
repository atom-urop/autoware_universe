#include "autoware/stanley_lateral_controller/stanley_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include <tf2/utils.h>

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

TEST(StanleyUtilsTest, CalcNearestPoseInterpStanleyEmptyTrajectory)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  geometry_msgs::msg::Pose self_pose;

  size_t nearest_idx = 999;

  const auto nearest_pose = calcNearestPoseInterpStanley(
    trajectory,
    self_pose,
    3.0,
    M_PI_2,
    nearest_idx);

  EXPECT_EQ(nearest_idx, 0u);

  EXPECT_NEAR(nearest_pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(nearest_pose.position.y, 0.0, 1e-9);

  EXPECT_NEAR(nearest_pose.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(nearest_pose.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(nearest_pose.orientation.z, 0.0, 1e-9);
  EXPECT_NEAR(nearest_pose.orientation.w, 1.0, 1e-9);
}

TEST(StanleyUtilsTest, CalcNearestPoseInterpStanleySinglePoint)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  autoware_planning_msgs::msg::TrajectoryPoint point;

  // Single trajectory point
  point.pose.position.x = 10.0;
  point.pose.position.y = 5.0;

  // Yaw = 0
  point.pose.orientation.x = 0.0;
  point.pose.orientation.y = 0.0;
  point.pose.orientation.z = 0.0;
  point.pose.orientation.w = 1.0;

  trajectory.points.push_back(point);

  geometry_msgs::msg::Pose self_pose;

  self_pose.position.x = 10.5;
  self_pose.position.y = 5.2;

  size_t nearest_idx = 999;

  const auto nearest_pose = calcNearestPoseInterpStanley(
    trajectory,
    self_pose,
    3.0,
    M_PI_2,
    nearest_idx);

  // With a single trajectory point, the nearest index must be 0.
  EXPECT_EQ(nearest_idx, 0u);

  // The function must return exactly the only trajectory point.
  EXPECT_NEAR(nearest_pose.position.x, 10.0, 1e-9);
  EXPECT_NEAR(nearest_pose.position.y, 5.0, 1e-9);

  EXPECT_NEAR(nearest_pose.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(nearest_pose.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(nearest_pose.orientation.z, 0.0, 1e-9);
  EXPECT_NEAR(nearest_pose.orientation.w, 1.0, 1e-9);
}

TEST(StanleyUtilsTest, CalcNearestPoseInterpStanleyStraightTrajectory)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  // Point 0: (0, 0), yaw = 0
  autoware_planning_msgs::msg::TrajectoryPoint point0;
  point0.pose.position.x = 0.0;
  point0.pose.position.y = 0.0;
  point0.pose.orientation.x = 0.0;
  point0.pose.orientation.y = 0.0;
  point0.pose.orientation.z = 0.0;
  point0.pose.orientation.w = 1.0;

  // Point 1: (10, 0), yaw = 0
  autoware_planning_msgs::msg::TrajectoryPoint point1;
  point1.pose.position.x = 10.0;
  point1.pose.position.y = 0.0;
  point1.pose.orientation.x = 0.0;
  point1.pose.orientation.y = 0.0;
  point1.pose.orientation.z = 0.0;
  point1.pose.orientation.w = 1.0;

  trajectory.points.push_back(point0);
  trajectory.points.push_back(point1);

  // Ego is 3 m ahead and 2 m laterally from the trajectory.
  geometry_msgs::msg::Pose self_pose;
  self_pose.position.x = 3.0;
  self_pose.position.y = 2.0;

  size_t nearest_idx = 999;

  const auto nearest_pose = calcNearestPoseInterpStanley(
    trajectory,
    self_pose,
    3.0,
    M_PI_2,
    nearest_idx);

  // The nearest trajectory point is point 0.
  EXPECT_EQ(nearest_idx, 0u);

  // Projection onto the segment gives x = 3, y = 0.
  EXPECT_NEAR(nearest_pose.position.x, 3.0, 1e-9);
  EXPECT_NEAR(nearest_pose.position.y, 0.0, 1e-9);

  // Straight trajectory -> yaw = 0.
  EXPECT_NEAR(nearest_pose.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(nearest_pose.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(nearest_pose.orientation.z, 0.0, 1e-9);
  EXPECT_NEAR(nearest_pose.orientation.w, 1.0, 1e-9);
}

TEST(StanleyUtilsTest, CalcNearestPoseInterpStanleyDiagonalTrajectory)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  // Point 0: (0, 0), yaw = pi/4
  autoware_planning_msgs::msg::TrajectoryPoint point0;
  point0.pose.position.x = 0.0;
  point0.pose.position.y = 0.0;

  const double yaw = M_PI / 4.0;

  point0.pose.orientation.x = 0.0;
  point0.pose.orientation.y = 0.0;
  point0.pose.orientation.z = std::sin(yaw / 2.0);
  point0.pose.orientation.w = std::cos(yaw / 2.0);

  // Point 1: (5, 5), yaw = pi/4
  autoware_planning_msgs::msg::TrajectoryPoint point1;
  point1.pose.position.x = 5.0;
  point1.pose.position.y = 5.0;
  point1.pose.orientation.x = 0.0;
  point1.pose.orientation.y = 0.0;
  point1.pose.orientation.z = std::sin(yaw / 2.0);
  point1.pose.orientation.w = std::cos(yaw / 2.0);

  trajectory.points.push_back(point0);
  trajectory.points.push_back(point1);

  // Ego position.
  //
  // Projection onto the line y = x is (2, 2).
  geometry_msgs::msg::Pose self_pose;
  self_pose.position.x = 3.0;
  self_pose.position.y = 1.0;

  size_t nearest_idx = 999;

  const auto nearest_pose = calcNearestPoseInterpStanley(
    trajectory,
    self_pose,
    3.0,
    M_PI_2,
    nearest_idx);

  // Point 0 is the nearest trajectory point.
  EXPECT_EQ(nearest_idx, 0u);

  // Projection of (3,1) onto y = x:
  // [2, 2]
  EXPECT_NEAR(nearest_pose.position.x, 2.0, 1e-9);
  EXPECT_NEAR(nearest_pose.position.y, 2.0, 1e-9);

  // The interpolated yaw must remain pi/4.
  EXPECT_NEAR(
    tf2::getYaw(nearest_pose.orientation),
    M_PI / 4.0,
    1e-9);
}

TEST(StanleyUtilsTest, CalcNearestPoseInterpStanleyYawWrapAround)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  const double yaw_prev = 179.0 * M_PI / 180.0;
  const double yaw_next = -179.0 * M_PI / 180.0;

  autoware_planning_msgs::msg::TrajectoryPoint point0;
  point0.pose.position.x = 0.0;
  point0.pose.position.y = 0.0;
  point0.pose.orientation.z = std::sin(yaw_prev / 2.0);
  point0.pose.orientation.w = std::cos(yaw_prev / 2.0);

  autoware_planning_msgs::msg::TrajectoryPoint point1;
  point1.pose.position.x = 10.0;
  point1.pose.position.y = 0.0;
  point1.pose.orientation.z = std::sin(yaw_next / 2.0);
  point1.pose.orientation.w = std::cos(yaw_next / 2.0);

  trajectory.points.push_back(point0);
  trajectory.points.push_back(point1);

  // Projection halfway along the trajectory.
  geometry_msgs::msg::Pose self_pose;
  self_pose.position.x = 5.0;
  self_pose.position.y = 0.0;

  size_t nearest_idx = 999;

  const auto nearest_pose = calcNearestPoseInterpStanley(
    trajectory,
    self_pose,
    3.0,
    M_PI_2,
    nearest_idx);

  EXPECT_EQ(nearest_idx, 0u);

  EXPECT_NEAR(nearest_pose.position.x, 5.0, 1e-9);
  EXPECT_NEAR(nearest_pose.position.y, 0.0, 1e-9);

  const double nearest_yaw = tf2::getYaw(nearest_pose.orientation);

  // The correct interpolated yaw is ±pi, not 0.
  EXPECT_NEAR(std::abs(nearest_yaw), M_PI, 1e-9);
}

TEST(StanleyUtilsTest, CalcNearestPoseInterpStanleyTrajectoryEdges)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  for (int i = 0; i < 5; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;

    point.pose.position.x = static_cast<double>(i);
    point.pose.position.y = 0.0;

    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;

    trajectory.points.push_back(point);
  }

  // ----------------------------------------------------------
  // Case 1: nearest point is the first trajectory point
  // ----------------------------------------------------------

  geometry_msgs::msg::Pose self_pose_start;
  self_pose_start.position.x = 0.0;
  self_pose_start.position.y = 0.5;

  size_t nearest_idx_start = 999;

  const auto nearest_pose_start = calcNearestPoseInterpStanley(
    trajectory,
    self_pose_start,
    3.0,
    M_PI_2,
    nearest_idx_start);

  EXPECT_EQ(nearest_idx_start, 0u);

  EXPECT_NEAR(nearest_pose_start.position.x, 0.0, 1e-9);
  EXPECT_NEAR(nearest_pose_start.position.y, 0.0, 1e-9);

  // ----------------------------------------------------------
  // Case 2: nearest point is the last trajectory point
  // ----------------------------------------------------------

  geometry_msgs::msg::Pose self_pose_end;
  self_pose_end.position.x = 4.0;
  self_pose_end.position.y = 0.5;

  size_t nearest_idx_end = 999;

  const auto nearest_pose_end = calcNearestPoseInterpStanley(
    trajectory,
    self_pose_end,
    3.0,
    M_PI_2,
    nearest_idx_end);

  EXPECT_EQ(nearest_idx_end, 4u);

  EXPECT_NEAR(nearest_pose_end.position.x, 4.0, 1e-9);
  EXPECT_NEAR(nearest_pose_end.position.y, 0.0, 1e-9);
}

TEST(StanleyUtilsTest, CalcNearestPoseInterpStanleyDistanceThreshold)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  for (int i = 0; i < 5; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;

    point.pose.position.x = static_cast<double>(i) * 2.0;
    point.pose.position.y = 0.0;

    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;

    trajectory.points.push_back(point);
  }

  geometry_msgs::msg::Pose self_pose;

  // Close to trajectory point 2: (4, 0)
  self_pose.position.x = 4.0;
  self_pose.position.y = 1.0;

  size_t nearest_idx = 999;

  const auto nearest_pose = calcNearestPoseInterpStanley(
    trajectory,
    self_pose,
    3.0,
    M_PI_2,
    nearest_idx);

  // The closest trajectory point is point 2.
  EXPECT_EQ(nearest_idx, 2u);

  // The interpolated pose lies on the trajectory.
  EXPECT_NEAR(nearest_pose.position.x, 4.0, 1e-9);
  EXPECT_NEAR(nearest_pose.position.y, 0.0, 1e-9);

  EXPECT_NEAR(
    tf2::getYaw(nearest_pose.orientation),
    0.0,
    1e-9);
}

}  // namespace autoware::motion::control::stanley_lateral_controller