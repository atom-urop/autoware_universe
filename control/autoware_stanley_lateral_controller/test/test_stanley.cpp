#include "autoware/stanley_lateral_controller/stanley.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "autoware/stanley_lateral_controller/stanley_utils.hpp"

namespace autoware::motion::control::stanley_lateral_controller
{

TEST(StanleyTest, GetDataPredictedLateralError)
{
  Stanley stanley(
    0.1,  // traj_resample_dist [m]
    2.0,  // curvature_calculation_distance [m]
    2.0,  // ATOM wheelbase [m]
    0.7); // ATOM max steer angle [rad]

  autoware_planning_msgs::msg::Trajectory trajectory;

  // Reference trajectory: straight line along x-axis.
  for (int i = 0; i < 2; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;

    point.pose.position.x = static_cast<double>(i) * 10.0;
    point.pose.position.y = 0.0;

    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;

    trajectory.points.push_back(point);
  }

  // Current front axle.
  nav_msgs::msg::Odometry current_front_odometry;
  current_front_odometry.pose.pose.position.x = 2.0;
  current_front_odometry.pose.pose.position.y = -1.0;
  current_front_odometry.pose.pose.orientation.w = 1.0;

  // Predicted front axle.
  // It is 1 m to the right of the reference trajectory.
  nav_msgs::msg::Odometry predicted_front_odometry;
  predicted_front_odometry.pose.pose.position.x = 3.0;
  predicted_front_odometry.pose.pose.position.y = -1.0;
  predicted_front_odometry.pose.pose.orientation.w = 1.0;

  StanleyData data;

  const auto result = stanley.getData(
    trajectory,
    current_front_odometry,
    predicted_front_odometry,
    data);

  EXPECT_TRUE(result.result);

  // For a straight trajectory with yaw = 0:
  // lateral_error = -err_y
  //
  // err_y = -1 m -> lateral_error = +1 m.
  EXPECT_NEAR(data.lateral_error, 1.0, 1e-9);
}

}  // namespace autoware::motion::control::stanley_lateral_controller