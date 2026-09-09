#include "autoware/stanley_lateral_controller/stanley.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

#include "autoware/stanley_lateral_controller/stanley_utils.hpp"


std::pair<std::vector<double>, std::vector<double>> loadStanleyLUT()
{
  const auto package_path =
    ament_index_cpp::get_package_share_directory(
      "autoware_stanley_lateral_controller");

  const auto yaml_path =
    package_path + "/param/stanley.param.yaml";

  const auto config = YAML::LoadFile(yaml_path);

  const auto parameters = config["/**"]["ros__parameters"];

  const auto k_ref_LUT =
    parameters["k_ref_LUT"].as<std::vector<double>>();

  const auto rr_LUT =
    parameters["rr_LUT"].as<std::vector<double>>();

  return {k_ref_LUT, rr_LUT};
}

const auto [test_k_ref_LUT, test_rr_LUT] = loadStanleyLUT();

namespace autoware::motion::control::stanley_lateral_controller
{

TEST(StanleyTest, LoadStanleyLUT)
{
  const auto [k_ref_LUT, rr_LUT] = loadStanleyLUT();

  ASSERT_FALSE(k_ref_LUT.empty());
  ASSERT_FALSE(rr_LUT.empty());

  ASSERT_EQ(k_ref_LUT.size(), rr_LUT.size());

  EXPECT_NEAR(k_ref_LUT.front(), 0.0, 1e-12);
  EXPECT_NEAR(k_ref_LUT.back(), 0.644217687237691, 1e-12);

  EXPECT_NEAR(rr_LUT.front(), 0.0, 1e-12);
  EXPECT_NEAR(rr_LUT.back(), -1.0, 1e-12);
}

TEST(StanleyTest, GetDataPredictedLateralError)
{
  Stanley stanley(
    0.1,  // traj_resample_dist [m]
    2.0,  // curvature_calculation_distance [m]
    2.0,  // ATOM wheelbase [m]
    0.7,  // ATOM max steer angle [rad]
    test_k_ref_LUT,
    test_rr_LUT); 

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

TEST(StanleyTest, GetDataStraightTrajectory)
{
  Stanley stanley(
    0.1,  // traj_resample_dist [m]
    2.0,  // curvature_calculation_distance [m]
    2.0,  // wheelbase [m]
    0.7,
    test_k_ref_LUT,
    test_rr_LUT); 

  // ------------------------------------------------------------
  // Reference trajectory
  // ------------------------------------------------------------

  autoware_planning_msgs::msg::Trajectory trajectory;

  for (int i = 0; i < 21; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;

    point.pose.position.x = static_cast<double>(i);
    point.pose.position.y = 0.0;

    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;

    trajectory.points.push_back(point);
  }

  // ------------------------------------------------------------
  // Current front axle odometry
  // ------------------------------------------------------------

  nav_msgs::msg::Odometry current_front_odometry;

  current_front_odometry.pose.pose.position.x = 5.0;
  current_front_odometry.pose.pose.position.y = -1.0;

  current_front_odometry.pose.pose.orientation.x = 0.0;
  current_front_odometry.pose.pose.orientation.y = 0.0;
  current_front_odometry.pose.pose.orientation.z = 0.0;
  current_front_odometry.pose.pose.orientation.w = 1.0;

  // ------------------------------------------------------------
  // Predicted front axle odometry
  // ------------------------------------------------------------

  nav_msgs::msg::Odometry predicted_front_odometry;

  predicted_front_odometry.pose.pose.position.x = 6.0;
  predicted_front_odometry.pose.pose.position.y = -1.0;

  predicted_front_odometry.pose.pose.orientation.x = 0.0;
  predicted_front_odometry.pose.pose.orientation.y = 0.0;
  predicted_front_odometry.pose.pose.orientation.z = 0.0;
  predicted_front_odometry.pose.pose.orientation.w = 1.0;

  // ------------------------------------------------------------
  // Execute getData()
  // ------------------------------------------------------------

  StanleyData data;

  const auto result = stanley.getData(
    trajectory,
    current_front_odometry,
    predicted_front_odometry,
    data);

  // ------------------------------------------------------------
  // Checks
  // ------------------------------------------------------------

  EXPECT_TRUE(result.result);

  // The predicted vehicle is 1 m below the trajectory.
  //
  // With the Stanley convention used by
  // calcLateralErrorStanley(), this gives +1 m.
  EXPECT_NEAR(
    data.lateral_error,
    1.0,
    1e-9);

  // The reference trajectory is perfectly straight.
  EXPECT_NEAR(
    data.reference_curvature,
    0.0,
    1e-9);
}

TEST(StanleyTest, GetDataSelectsLargestAbsoluteCurvature)
{
  const double ds = 0.1;
  const double curvature_calculation_distance = 2.0;
  const double wheel_base = 2.0;
  const double max_steer_angle = 0.7;

  Stanley stanley(
    ds,
    curvature_calculation_distance,
    wheel_base,
    max_steer_angle,
    test_k_ref_LUT,
    test_rr_LUT); 

  autoware_planning_msgs::msg::Trajectory trajectory;

  // ------------------------------------------------------------
  // First circular arc
  // R1 = 10 m -> kappa1 = 0.1 1/m
  // ------------------------------------------------------------

  const double R1 = 10.0;
  const int N1 = 81;  // s = 0 ... 8 m

  for (int i = 0; i < N1; ++i) {
    const double s = i * ds;
    const double theta = s / R1;

    autoware_planning_msgs::msg::TrajectoryPoint point;

    point.pose.position.x = R1 * std::sin(theta);
    point.pose.position.y = R1 * (1.0 - std::cos(theta));

    // Tangent heading
    point.pose.orientation.z = std::sin(theta / 2.0);
    point.pose.orientation.w = std::cos(theta / 2.0);

    trajectory.points.push_back(point);
  }

  // ------------------------------------------------------------
  // Second circular arc
  // R2 = 5 m -> kappa2 = 0.2 1/m
  //
  // The second arc starts from the end of the first arc and
  // preserves position and tangent continuity.
  // ------------------------------------------------------------

  const double R2 = 5.0;
  const int N2 = 80;

  const double s1 = (N1 - 1) * ds;
  const double theta1 = s1 / R1;

  const double x1 = R1 * std::sin(theta1);
  const double y1 = R1 * (1.0 - std::cos(theta1));

  const double tx = std::cos(theta1);
  const double ty = std::sin(theta1);

  const double nx = -std::sin(theta1);
  const double ny = std::cos(theta1);

  for (int i = 1; i <= N2; ++i) {
    const double s = i * ds;
    const double phi = s / R2;

    autoware_planning_msgs::msg::TrajectoryPoint point;

    point.pose.position.x =
      x1 + R2 * (tx * std::sin(phi) + nx * (1.0 - std::cos(phi)));

    point.pose.position.y =
      y1 + R2 * (ty * std::sin(phi) + ny * (1.0 - std::cos(phi)));

    const double heading = theta1 + phi;

    point.pose.orientation.z =
      std::sin(heading / 2.0);

    point.pose.orientation.w =
      std::cos(heading / 2.0);

    trajectory.points.push_back(point);
  }

  ASSERT_EQ(trajectory.points.size(), 161u);

  // ------------------------------------------------------------
  // Current odometry: inside first arc
  // index = 40 -> s = 4 m
  // ------------------------------------------------------------

  const size_t current_idx = 40;

  nav_msgs::msg::Odometry current_front_odometry;

  current_front_odometry.pose.pose =
    trajectory.points.at(current_idx).pose;

  // ------------------------------------------------------------
  // Predicted odometry: inside second arc
  // index = 120 -> second arc, 4 m after transition
  // ------------------------------------------------------------

  const size_t predicted_idx = 120;

  nav_msgs::msg::Odometry predicted_front_odometry;

  predicted_front_odometry.pose.pose =
    trajectory.points.at(predicted_idx).pose;

  // ------------------------------------------------------------
  // Execute getData()
  // ------------------------------------------------------------

  StanleyData data;

  const auto result = stanley.getData(
    trajectory,
    current_front_odometry,
    predicted_front_odometry,
    data);

  ASSERT_TRUE(result.result);

  // ------------------------------------------------------------
  // Independently obtain the curvature vector.
  //
  // This function has already been verified against MATLAB/Simulink.
  // ------------------------------------------------------------

  const auto curvature_vector = calcCurvatureVectorStanley(
    trajectory,
    ds,
    curvature_calculation_distance);

  ASSERT_EQ(
    curvature_vector.size(),
    trajectory.points.size());

  const double current_curvature =
    curvature_vector.at(current_idx);

  const double predicted_curvature =
    curvature_vector.at(predicted_idx);

  // ------------------------------------------------------------
  // Verify that the predicted point really has the larger
  // absolute curvature.
  // ------------------------------------------------------------

  EXPECT_LT(
    std::abs(current_curvature),
    std::abs(predicted_curvature));

  // The values should also be close to the theoretical values.
  EXPECT_NEAR(
    current_curvature,
    0.1,
    1e-3);

  EXPECT_NEAR(
    predicted_curvature,
    0.2,
    1e-3);

  // ------------------------------------------------------------
  // Main assertion:
  //
  // getData() must select the curvature with the largest
  // absolute value.
  // ------------------------------------------------------------

  EXPECT_NEAR(
    data.reference_curvature,
    predicted_curvature,
    1e-9);

  // ------------------------------------------------------------
  // Lateral error sanity check.
  //
  // Both odometries are exactly on the trajectory.
  // ------------------------------------------------------------

  EXPECT_NEAR(
    data.lateral_error,
    0.0,
    1e-9);
}

TEST(StanleyTest, GetDataSelectsCurrentCurvatureWhenLarger)
{
  const double ds = 0.1;
  const double curvature_calculation_distance = 2.0;
  const double wheel_base = 2.0;
  const double max_steer_angle = 0.7;

  Stanley stanley(
    ds,
    curvature_calculation_distance,
    wheel_base,
    max_steer_angle,
    test_k_ref_LUT,
    test_rr_LUT); 

  autoware_planning_msgs::msg::Trajectory trajectory;

  // ------------------------------------------------------------
  // First circular arc
  // R1 = 10 m -> kappa1 = 0.1 1/m
  // ------------------------------------------------------------

  const double R1 = 10.0;
  const int N1 = 81;

  for (int i = 0; i < N1; ++i) {
    const double s = i * ds;
    const double theta = s / R1;

    autoware_planning_msgs::msg::TrajectoryPoint point;

    point.pose.position.x =
      R1 * std::sin(theta);

    point.pose.position.y =
      R1 * (1.0 - std::cos(theta));

    point.pose.orientation.z =
      std::sin(theta / 2.0);

    point.pose.orientation.w =
      std::cos(theta / 2.0);

    trajectory.points.push_back(point);
  }

  // ------------------------------------------------------------
  // Second circular arc
  // R2 = 5 m -> kappa2 = 0.2 1/m
  // ------------------------------------------------------------

  const double R2 = 5.0;

  const double s1 = (N1 - 1) * ds;
  const double theta1 = s1 / R1;

  const double x1 =
    R1 * std::sin(theta1);

  const double y1 =
    R1 * (1.0 - std::cos(theta1));

  const double tx = std::cos(theta1);
  const double ty = std::sin(theta1);

  const double nx = -std::sin(theta1);
  const double ny = std::cos(theta1);

  const int N2 = 80;

  for (int i = 1; i <= N2; ++i) {
    const double s = i * ds;
    const double phi = s / R2;

    autoware_planning_msgs::msg::TrajectoryPoint point;

    point.pose.position.x =
      x1 +
      R2 * (
        tx * std::sin(phi) +
        nx * (1.0 - std::cos(phi)));

    point.pose.position.y =
      y1 +
      R2 * (
        ty * std::sin(phi) +
        ny * (1.0 - std::cos(phi)));

    const double heading = theta1 + phi;

    point.pose.orientation.z =
      std::sin(heading / 2.0);

    point.pose.orientation.w =
      std::cos(heading / 2.0);

    trajectory.points.push_back(point);
  }

  ASSERT_EQ(trajectory.points.size(), 161u);

  // ------------------------------------------------------------
  // Current -> second arc, kappa ~= 0.2
  // ------------------------------------------------------------

  const size_t current_idx = 120;

  nav_msgs::msg::Odometry current_front_odometry;

  current_front_odometry.pose.pose =
    trajectory.points.at(current_idx).pose;

  // ------------------------------------------------------------
  // Predicted -> first arc, kappa ~= 0.1
  // ------------------------------------------------------------

  const size_t predicted_idx = 40;

  nav_msgs::msg::Odometry predicted_front_odometry;

  predicted_front_odometry.pose.pose =
    trajectory.points.at(predicted_idx).pose;

  // ------------------------------------------------------------
  // Execute getData()
  // ------------------------------------------------------------

  StanleyData data;

  const auto result = stanley.getData(
    trajectory,
    current_front_odometry,
    predicted_front_odometry,
    data);

  ASSERT_TRUE(result.result);

  // ------------------------------------------------------------
  // Independently obtain curvature vector
  // ------------------------------------------------------------

  const auto curvature_vector =
    calcCurvatureVectorStanley(
      trajectory,
      ds,
      curvature_calculation_distance);

  ASSERT_EQ(
    curvature_vector.size(),
    trajectory.points.size());

  const double current_curvature =
    curvature_vector.at(current_idx);

  const double predicted_curvature =
    curvature_vector.at(predicted_idx);

  // ------------------------------------------------------------
  // Verify that current curvature is larger
  // ------------------------------------------------------------

  EXPECT_GT(
    std::abs(current_curvature),
    std::abs(predicted_curvature));

  EXPECT_NEAR(
    current_curvature,
    0.2,
    1e-3);

  EXPECT_NEAR(
    predicted_curvature,
    0.1,
    1e-3);

  // ------------------------------------------------------------
  // Main assertion:
  // getData() must select current curvature.
  // ------------------------------------------------------------

  EXPECT_NEAR(
    data.reference_curvature,
    current_curvature,
    1e-9);

  // Both odometries are exactly on the reference trajectory.
  EXPECT_NEAR(
    data.lateral_error,
    0.0,
    1e-9);
}

TEST(StanleyTest, GetDataSaturatesCurvature)
{
  const double ds = 0.1;
  const double curvature_calculation_distance = 2.0;
  const double wheel_base = 2.0;
  const double max_steer_angle = 0.7;

  Stanley stanley(
    ds,
    curvature_calculation_distance,
    wheel_base,
    max_steer_angle,
    test_k_ref_LUT,
    test_rr_LUT); 

  autoware_planning_msgs::msg::Trajectory trajectory;

  // ------------------------------------------------------------
  // Circular trajectory
  //
  // R = 1 m -> kappa = 1.0 1/m
  // ------------------------------------------------------------

  const double R = 1.0;
  const int N = 101;

  for (int i = 0; i < N; ++i) {
    const double s = i * ds;
    const double theta = s / R;

    autoware_planning_msgs::msg::TrajectoryPoint point;

    point.pose.position.x =
      R * std::sin(theta);

    point.pose.position.y =
      R * (1.0 - std::cos(theta));

    point.pose.orientation.z =
      std::sin(theta / 2.0);

    point.pose.orientation.w =
      std::cos(theta / 2.0);

    trajectory.points.push_back(point);
  }

  ASSERT_EQ(
    trajectory.points.size(),
    101u);

  // Choose a point well inside the arc.
  const size_t current_idx = 50;
  const size_t predicted_idx = 70;

  nav_msgs::msg::Odometry current_front_odometry;

  current_front_odometry.pose.pose =
    trajectory.points.at(current_idx).pose;

  nav_msgs::msg::Odometry predicted_front_odometry;

  predicted_front_odometry.pose.pose =
    trajectory.points.at(predicted_idx).pose;

  // ------------------------------------------------------------
  // Execute getData()
  // ------------------------------------------------------------

  StanleyData data;

  const auto result = stanley.getData(
    trajectory,
    current_front_odometry,
    predicted_front_odometry,
    data);

  ASSERT_TRUE(result.result);

  // ------------------------------------------------------------
  // Independently obtain raw curvature
  // ------------------------------------------------------------

  const auto curvature_vector =
    calcCurvatureVectorStanley(
      trajectory,
      ds,
      curvature_calculation_distance);

  const double raw_curvature =
    curvature_vector.at(current_idx);

  // The raw curvature must be above the allowed maximum.
  EXPECT_GT(
    std::abs(raw_curvature),
    0.7);

  // ------------------------------------------------------------
  // Compute the expected curvature limit independently
  // ------------------------------------------------------------

  const double curvature_max =
    std::sin(2.0 * max_steer_angle) /
    (wheel_base * std::cos(-max_steer_angle));

  // ------------------------------------------------------------
  // Main assertion:
  //
  // getData() must saturate the curvature.
  // ------------------------------------------------------------

  EXPECT_NEAR(
    data.reference_curvature,
    curvature_max,
    1e-9);

  // Verify that saturation actually occurred.
  EXPECT_LT(
    std::abs(data.reference_curvature),
    std::abs(raw_curvature));

  EXPECT_NEAR(
    data.lateral_error,
    0.0,
    1e-3);
}

TEST(StanleyTest, GetDataPreservesCurvatureSignWhenSaturated)
{
  const double ds = 0.1;
  const double curvature_calculation_distance = 2.0;
  const double wheel_base = 2.0;
  const double max_steer_angle = 0.7;

  Stanley stanley(
    ds,
    curvature_calculation_distance,
    wheel_base,
    max_steer_angle,
    test_k_ref_LUT,
    test_rr_LUT); 

  autoware_planning_msgs::msg::Trajectory trajectory;

  // ------------------------------------------------------------
  // Clockwise circular trajectory
  //
  // R = 1 m -> kappa = -1.0 1/m
  // ------------------------------------------------------------

  const double R = 1.0;
  const int N = 61;

  for (int i = 0; i < N; ++i) {
    const double s = i * ds;
    const double theta = s / R;

    autoware_planning_msgs::msg::TrajectoryPoint point;

    point.pose.position.x =
      R * std::sin(theta);

    point.pose.position.y =
      -R * (1.0 - std::cos(theta));

    const double heading = -theta;

    point.pose.orientation.z =
      std::sin(heading / 2.0);

    point.pose.orientation.w =
      std::cos(heading / 2.0);

    trajectory.points.push_back(point);
  }

  ASSERT_EQ(
    trajectory.points.size(),
    61u);

  const size_t test_idx = 30;

  nav_msgs::msg::Odometry current_front_odometry;

  current_front_odometry.pose.pose =
    trajectory.points.at(test_idx).pose;

  nav_msgs::msg::Odometry predicted_front_odometry;

  predicted_front_odometry.pose.pose =
    trajectory.points.at(test_idx).pose;

  // ------------------------------------------------------------
  // Execute getData()
  // ------------------------------------------------------------

  StanleyData data;

  const auto result = stanley.getData(
    trajectory,
    current_front_odometry,
    predicted_front_odometry,
    data);

  std::cout
  << "test_idx = " << test_idx
  << ", test_x = "
  << trajectory.points.at(test_idx).pose.position.x
  << ", test_y = "
  << trajectory.points.at(test_idx).pose.position.y
  << std::endl;

  ASSERT_TRUE(result.result);

  size_t nearest_idx = 0;

  const auto nearest_pose = calcNearestPoseInterpStanley(
    trajectory,
    current_front_odometry.pose.pose,
    stanley.ego_nearest_dist_threshold,
    stanley.ego_nearest_yaw_threshold,
    nearest_idx);

  std::cout
    << "test_idx = " << test_idx
    << ", nearest_idx = " << nearest_idx
    << std::endl;
  
  (void)nearest_pose;

  // ------------------------------------------------------------
  // Independently obtain raw curvature
  // ------------------------------------------------------------

  const auto curvature_vector =
    calcCurvatureVectorStanley(
      trajectory,
      ds,
      curvature_calculation_distance);

  const double raw_curvature =
    curvature_vector.at(test_idx);

  // Raw curvature must be negative and larger than the limit
  // in absolute value.
  EXPECT_LT(
    raw_curvature,
    -0.7);

  // ------------------------------------------------------------
  // Compute curvature limit
  // ------------------------------------------------------------

  const double curvature_max =
    std::sin(2.0 * max_steer_angle) /
    (wheel_base * std::cos(-max_steer_angle));

  // ------------------------------------------------------------
  // Main assertions:
  //
  // 1. Saturation occurs.
  // 2. The sign remains negative.
  // ------------------------------------------------------------

  EXPECT_NEAR(
    data.reference_curvature,
    -curvature_max,
    1e-9);

  EXPECT_LT(
    data.reference_curvature,
    0.0);

  EXPECT_LT(
    std::abs(data.reference_curvature),
    std::abs(raw_curvature));

  EXPECT_NEAR(
    data.lateral_error,
    0.0,
    1e-9);
}

}  // namespace autoware::motion::control::stanley_lateral_controller