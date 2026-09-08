#include "autoware/stanley_lateral_controller/stanley_utils.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>

#include <cmath>

#include <tf2/utils.h>

#include <utility>

#include <vector>

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

TEST(StanleyUtilsTest, CalcLateralErrorStanleyStraightTrajectory)
{
  nav_msgs::msg::Odometry ego_odometry;
  geometry_msgs::msg::Pose nearest_pose;

  // Reference point at (0, 0), yaw = 0.
  nearest_pose.position.x = 0.0;
  nearest_pose.position.y = 0.0;
  nearest_pose.orientation.x = 0.0;
  nearest_pose.orientation.y = 0.0;
  nearest_pose.orientation.z = 0.0;
  nearest_pose.orientation.w = 1.0;

  // Ego front axle at (0, -1).
  // The vehicle is on the right side of the trajectory.
  ego_odometry.pose.pose.position.x = 0.0;
  ego_odometry.pose.pose.position.y = -1.0;

  const double lateral_error =
    calcLateralErrorStanley(
      ego_odometry,
      nearest_pose);

  // For ref_yaw = 0:
  // lateral_error = -err_y
  // err_y = -1 -> lateral_error = +1
  EXPECT_NEAR(lateral_error, 1.0, 1e-9);
}

TEST(StanleyUtilsTest, CalcLateralErrorStanleyStraightTrajectoryLeft)
{
  nav_msgs::msg::Odometry ego_odometry;
  geometry_msgs::msg::Pose nearest_pose;

  // Reference point at (0, 0), yaw = 0.
  nearest_pose.position.x = 0.0;
  nearest_pose.position.y = 0.0;
  nearest_pose.orientation.x = 0.0;
  nearest_pose.orientation.y = 0.0;
  nearest_pose.orientation.z = 0.0;
  nearest_pose.orientation.w = 1.0;

  // Ego front axle at (0, +1).
  // The vehicle is on the left side of the trajectory.
  ego_odometry.pose.pose.position.x = 0.0;
  ego_odometry.pose.pose.position.y = 1.0;

  const double lateral_error =
    calcLateralErrorStanley(
      ego_odometry,
      nearest_pose);

  // For ref_yaw = 0:
  // lateral_error = -err_y
  // err_y = +1 -> lateral_error = -1
  EXPECT_NEAR(lateral_error, -1.0, 1e-9);
}

TEST(StanleyUtilsTest, CalcLateralErrorStanleyDiagonalTrajectory)
{
  nav_msgs::msg::Odometry ego_odometry;
  geometry_msgs::msg::Pose nearest_pose;

  const double ref_yaw = M_PI / 4.0;

  // Reference point at (0, 0), yaw = 45 deg.
  nearest_pose.position.x = 0.0;
  nearest_pose.position.y = 0.0;
  nearest_pose.orientation.x = 0.0;
  nearest_pose.orientation.y = 0.0;
  nearest_pose.orientation.z = std::sin(ref_yaw / 2.0);
  nearest_pose.orientation.w = std::cos(ref_yaw / 2.0);

  // Ego is displaced by 1 m to the right of the trajectory.
  //
  // Right-hand normal to a trajectory with yaw = pi/4:
  //
  // [ sin(yaw) ]
  // [-cos(yaw) ]
  //
  // Therefore:
  // ego = [sin(pi/4), -cos(pi/4)]
  ego_odometry.pose.pose.position.x =
    std::sin(ref_yaw);

  ego_odometry.pose.pose.position.y =
    -std::cos(ref_yaw);

  const double lateral_error =
    calcLateralErrorStanley(
      ego_odometry,
      nearest_pose);

  EXPECT_NEAR(lateral_error, 1.0, 1e-9);
}

TEST(StanleyUtilsTest, CalcLateralErrorStanleyVerticalTrajectory)
{
  nav_msgs::msg::Odometry ego_odometry;
  geometry_msgs::msg::Pose nearest_pose;

  const double ref_yaw = M_PI / 2.0;

  // Reference point at (0, 0), yaw = 90 deg.
  nearest_pose.position.x = 0.0;
  nearest_pose.position.y = 0.0;
  nearest_pose.orientation.x = 0.0;
  nearest_pose.orientation.y = 0.0;
  nearest_pose.orientation.z = std::sin(ref_yaw / 2.0);
  nearest_pose.orientation.w = std::cos(ref_yaw / 2.0);

  // Ego is displaced by 1 m to the right of the trajectory.
  //
  // For yaw = pi/2:
  //
  // [ sin(yaw) ]   [ 1 ]
  // [-cos(yaw) ] = [ 0 ]
  //
  // Therefore the ego position is (1, 0).
  ego_odometry.pose.pose.position.x = 1.0;
  ego_odometry.pose.pose.position.y = 0.0;

  const double lateral_error =
    calcLateralErrorStanley(
      ego_odometry,
      nearest_pose);

  // For ref_yaw = pi/2:
  // lateral_error = err_x = +1.
  EXPECT_NEAR(lateral_error, 1.0, 1e-9);
}

TEST(StanleyUtilsTest, CalcReferenceCurvature)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  // Three points on a circle of radius 5 m.
  // The middle point is at the top of the circle.
  const double R = 5.0;

  const double x0 = -R;
  const double y0 = 0.0;

  const double x1 = 0.0;
  const double y1 = R;

  const double x2 = R;
  const double y2 = 0.0;

  for (const auto & [x, y] :
       std::vector<std::pair<double, double>>{
         {x0, y0},
         {x1, y1},
         {x2, y2}})
  {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    trajectory.points.push_back(point);
  }

  const auto curvature_vector = calcCurvatureVectorStanley(
    trajectory,
    1.0,
    1.0);

  const double curvature = curvature_vector.at(1);

  EXPECT_NEAR(std::abs(curvature), 1.0 / R, 1e-9);
}

TEST(StanleyUtilsTest, CalcReferenceCurvatureSign)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  // Right turn: clockwise circular arc.
  const double R = 5.0;

  for (const auto & [x, y] :
       std::vector<std::pair<double, double>>{
         {-R, 0.0},
         {0.0, R},
         {R, 0.0}})
  {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    trajectory.points.push_back(point);
  }

  const auto curvature_vector = calcCurvatureVectorStanley(
    trajectory,
    1.0,
    1.0);

  const double curvature = curvature_vector.at(1);

  EXPECT_NEAR(curvature, -1.0 / R, 1e-9);
}

TEST(StanleyUtilsTest, CalcReferenceCurvatureFirstPoint)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  // Four points on a circular arc.
  const double R = 5.0;

  for (const auto & [x, y] :
       std::vector<std::pair<double, double>>{
         {-R, 0.0},
         {0.0, R},
         {R, 0.0},
         {2.0 * R, -R}})
  {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    trajectory.points.push_back(point);
  }

  const auto curvature_vector = calcCurvatureVectorStanley(
    trajectory,
    1.0,
    1.0);

  const double curvature_first = curvature_vector.at(0);

  const double curvature_second = curvature_vector.at(1);

  EXPECT_NEAR(curvature_first, curvature_second, 1e-9);
}

TEST(StanleyUtilsTest, CalcReferenceCurvatureLastPoint)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  // Four points on a circular arc.
  const double R = 5.0;

  for (const auto & [x, y] :
       std::vector<std::pair<double, double>>{
         {-R, 0.0},
         {0.0, R},
         {R, 0.0},
         {2.0 * R, -R}})
  {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    trajectory.points.push_back(point);
  }

  const size_t last_idx = trajectory.points.size() - 1;

  const auto curvature_vector = calcCurvatureVectorStanley(
    trajectory,
    1.0,
    1.0);

  const double curvature_last = curvature_vector.at(last_idx);

  const double curvature_previous = curvature_vector.at(last_idx - 1);

  EXPECT_NEAR(curvature_last, curvature_previous, 1e-9);
}

TEST(StanleyUtilsTest, CalcReferenceCurvatureIndexDistance)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  // Circular arc with radius 5 m.
  const double R = 5.0;

  // Points sampled every 1 m along the arc.
  for (int i = 0; i < 7; ++i) {
    const double theta =
      -M_PI / 2.0 + static_cast<double>(i) * M_PI / 6.0;

    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = R * std::cos(theta);
    point.pose.position.y = R * std::sin(theta);
    trajectory.points.push_back(point);
  }

  const auto curvature_vector_idx_dist_2 = calcCurvatureVectorStanley(
    trajectory,
    1.0,
    2.0);
  
  const auto curvature_vector_idx_dist_1 = calcCurvatureVectorStanley(
    trajectory,
    1.0,
    1.0);

  // curvature_calculation_distance = 2 m
  // traj_resample_dist = 1 m
  // -> idx_dist = 2
  const double curvature_idx_dist_2 = curvature_vector_idx_dist_2.at(3);

  // curvature_calculation_distance = 1 m
  // traj_resample_dist = 1 m
  // -> idx_dist = 1
  const double curvature_idx_dist_1 = curvature_vector_idx_dist_1.at(3);

  // Both calculations are performed on the same circular arc,
  // therefore the curvature should remain approximately 1/R.
  EXPECT_NEAR(std::abs(curvature_idx_dist_2), 1.0 / R, 1e-3);
  EXPECT_NEAR(std::abs(curvature_idx_dist_1), 1.0 / R, 1e-3);
}

TEST(StanleyUtilsTest, CalcReferenceCurvatureMaxIndexDistance)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  // Five points on a circular arc.
  const double R = 5.0;

  for (int i = 0; i < 5; ++i) {
    const double theta =
      -M_PI / 2.0 + static_cast<double>(i) * M_PI / 4.0;

    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = R * std::cos(theta);
    point.pose.position.y = R * std::sin(theta);
    trajectory.points.push_back(point);
  }

  // N = 5:
  // max_idx_dist = floor((5 - 1) / 2) = 2.
  //
  // Request an index distance of 10:
  // 10 / 1 = 10 -> must be clamped to 2.

  const auto curvature_vector = calcCurvatureVectorStanley(
    trajectory,
    1.0,
    10.0);

  const double curvature = curvature_vector.at(2);

  EXPECT_NEAR(std::abs(curvature), 1.0 / R, 1e-3);
}

TEST(StanleyUtilsTest, CalcReferenceCurvatureTooFewPoints)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  // Two points are not enough to calculate curvature.
  for (int i = 0; i < 2; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(i);
    point.pose.position.y = 0.0;
    trajectory.points.push_back(point);
  }

  const auto curvature_vector = calcCurvatureVectorStanley(
    trajectory,
    1.0,
    1.0);

  const double curvature = curvature_vector.at(0);

  EXPECT_DOUBLE_EQ(curvature, 0.0);
}

TEST(StanleyUtilsTest, CalcReferenceCurvatureVariableCurvature)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  // Straight section followed by a circular arc.
  //
  // The first three points are collinear, so the curvature at index 1
  // must be zero.
  //
  // The last three points belong to a circular arc with radius 5 m,
  // so the curvature at index 4 must be approximately 1/R.
  const double R = 5.0;

  const std::vector<std::pair<double, double>> points{
    {0.0, 0.0},
    {1.0, 0.0},
    {2.0, 0.0},
    {3.0, 0.0},
    {3.0 + R * std::sin(M_PI / 6.0), R * (1.0 - std::cos(M_PI / 6.0))},
    {3.0 + R * std::sin(M_PI / 3.0), R * (1.0 - std::cos(M_PI / 3.0))},
    {3.0 + R, R}
  };

  for (const auto & [x, y] : points) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    trajectory.points.push_back(point);
  }

  const auto curvature_vector = calcCurvatureVectorStanley(
    trajectory,
    1.0,
    1.0);

  const double curvature_straight = curvature_vector.at(1);

  const double curvature_curve = curvature_vector.at(5);

  EXPECT_NEAR(curvature_straight, 0.0, 1e-9);
  EXPECT_NEAR(std::abs(curvature_curve), 1.0 / R, 1e-3);
}


TEST(StanleyUtilsTest, CalcReferenceCurvatureRealTrajectoryDuplicates)
{
  autoware_planning_msgs::msg::Trajectory trajectory;

  const std::string file_path =
    "/mnt/ssd/autoware/src/universe/autoware_universe/"
    "control/autoware_stanley_lateral_controller/test/"
    "pointsLiveTraj_cpp.txt";

  std::ifstream file(file_path);

  ASSERT_TRUE(file.is_open())
    << "Failed to open file: " << file_path;

  std::string line;

  while (std::getline(file, line)) {
    std::stringstream ss(line);

    char c;
    double x;
    double y;

    ss >> c >> x >> c >> y >> c;

    ASSERT_FALSE(ss.fail())
      << "Failed to parse line: " << line;

    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;

    trajectory.points.push_back(point);
  }

  file.close();

  ASSERT_GE(trajectory.points.size(), 3u);

  const double traj_resample_dist = 0.1;
  const double curvature_calculation_distance = 2.0;

  const auto curvature_vector = calcCurvatureVectorStanley(
    trajectory,
    traj_resample_dist,
    curvature_calculation_distance);

  ASSERT_EQ(
    curvature_vector.size(),
    trajectory.points.size());

  // Export curvature results to CSV
  std::ofstream output_file(
    "/mnt/ssd/autoware/src/universe/autoware_universe/"
    "control/autoware_stanley_lateral_controller/test/"
    "curvature_cpp.csv");

  ASSERT_TRUE(output_file.is_open())
    << "Failed to open output file";

  output_file << "index,x,y,curvature\n";

  for (size_t i = 0; i < curvature_vector.size(); ++i) {
    output_file
      << i << ","
      << trajectory.points.at(i).pose.position.x << ","
      << trajectory.points.at(i).pose.position.y << ","
      << curvature_vector.at(i) << "\n";
  }

  output_file.close();

  SUCCEED();
}

}  // namespace autoware::motion::control::stanley_lateral_controller