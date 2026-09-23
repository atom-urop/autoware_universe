#include "autoware/path_optimizer/mpt_optimizer.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::path_optimizer
{

class MPTOptimizerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    const auto path_optimizer_package =
      ament_index_cpp::get_package_share_directory("autoware_path_optimizer");

    const auto test_utils_package =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");

    const auto vehicle_info_yaml =
      test_utils_package + "/config/test_vehicle_info.param.yaml";

    const auto common_yaml =
      test_utils_package + "/config/test_common.param.yaml";

    const auto nearest_search_yaml =
      test_utils_package + "/config/test_nearest_search.param.yaml";

    const auto path_optimizer_yaml =
      path_optimizer_package + "/config/path_optimizer.param.yaml";

    rclcpp::NodeOptions node_options;

    node_options.arguments(
      {
        "--ros-args",
        "--params-file", vehicle_info_yaml,
        "--params-file", common_yaml,
        "--params-file", nearest_search_yaml,
        "--params-file", path_optimizer_yaml
      });

    node_options.parameter_overrides(
      {
        rclcpp::Parameter("mpt.kinematics.model_type", "4ws"),
        rclcpp::Parameter("mpt.weight.steer_input_weight", 3.0),
        rclcpp::Parameter("mpt.weight.steer_rate_weight", 2.0),
      });

    node_ =
      std::make_shared<rclcpp::Node>("mpt_optimizer_test", node_options);

    vehicle_info_ =
      autoware::vehicle_info_utils::VehicleInfoUtils(*node_).getVehicleInfo();

    ego_nearest_param_ =
      EgoNearestParam(node_.get());

    traj_param_ =
      TrajectoryParam(node_.get());

    debug_data_ptr_ =
      std::make_shared<DebugData>();

    time_keeper_ =
      std::make_shared<autoware_utils::TimeKeeper>();

    optimizer_ = std::make_unique<MPTOptimizer>(
      node_.get(),
      false,
      ego_nearest_param_,
      vehicle_info_,
      traj_param_,
      debug_data_ptr_,
      time_keeper_);
  }

  std::shared_ptr<rclcpp::Node> node_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  EgoNearestParam ego_nearest_param_;
  TrajectoryParam traj_param_;
  std::shared_ptr<DebugData> debug_data_ptr_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
  std::unique_ptr<MPTOptimizer> optimizer_;
};

TEST_F(MPTOptimizerTest, SteeringRateMatrix4WS)
{
  constexpr size_t N_ref = 3;
  constexpr size_t D_u = 2;
  constexpr size_t N_u = (N_ref - 1) * D_u;

  std::vector<ReferencePoint> ref_points(N_ref);

  for (auto & ref_point : ref_points) {
    ref_point.normalized_avoidance_cost = 0.0;
  }

  std::vector<TrajectoryPoint> traj_points(N_ref);

  const auto value_matrix =
    optimizer_->calcValueMatrix(ref_points, traj_points);

  ASSERT_EQ(value_matrix.R.rows(), N_u);
  ASSERT_EQ(value_matrix.R.cols(), N_u);

  const double steer_input_weight =
    optimizer_->mpt_param_.steer_input_weight;

  const double steer_rate_weight =
    optimizer_->mpt_param_.steer_rate_weight;

  // Diagonal terms.
  EXPECT_DOUBLE_EQ(
    value_matrix.R.coeff(0, 0),
    steer_input_weight + steer_rate_weight);

  EXPECT_DOUBLE_EQ(
    value_matrix.R.coeff(1, 1),
    steer_input_weight + steer_rate_weight);

  EXPECT_DOUBLE_EQ(
    value_matrix.R.coeff(2, 2),
    steer_input_weight + steer_rate_weight);

  EXPECT_DOUBLE_EQ(
    value_matrix.R.coeff(3, 3),
    steer_input_weight + steer_rate_weight);

  // Front steering rate:
  // (delta_f1 - delta_f0)^2
  EXPECT_DOUBLE_EQ(
    value_matrix.R.coeff(0, 2),
    -steer_rate_weight);

  EXPECT_DOUBLE_EQ(
    value_matrix.R.coeff(2, 0),
    -steer_rate_weight);

  // Rear steering rate:
  // (delta_r1 - delta_r0)^2
  EXPECT_DOUBLE_EQ(
    value_matrix.R.coeff(1, 3),
    -steer_rate_weight);

  EXPECT_DOUBLE_EQ(
    value_matrix.R.coeff(3, 1),
    -steer_rate_weight);

  // There must be no front/rear coupling.
  EXPECT_DOUBLE_EQ(value_matrix.R.coeff(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(value_matrix.R.coeff(1, 0), 0.0);

  EXPECT_DOUBLE_EQ(value_matrix.R.coeff(1, 2), 0.0);
  EXPECT_DOUBLE_EQ(value_matrix.R.coeff(2, 1), 0.0);

  EXPECT_DOUBLE_EQ(value_matrix.R.coeff(2, 3), 0.0);
  EXPECT_DOUBLE_EQ(value_matrix.R.coeff(3, 2), 0.0);
}

TEST_F(MPTOptimizerTest, SteeringRateMatrix2WS)
{
  constexpr size_t N_ref = 3;
  constexpr size_t D_u = 1;
  constexpr size_t N_u = (N_ref - 1) * D_u;

  const auto package_path =
    ament_index_cpp::get_package_share_directory("autoware_path_optimizer");

  const auto test_utils_package =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");

  const auto vehicle_info_yaml =
    test_utils_package + "/config/test_vehicle_info.param.yaml";

  const auto common_yaml =
    test_utils_package + "/config/test_common.param.yaml";

  const auto nearest_search_yaml =
    test_utils_package + "/config/test_nearest_search.param.yaml";

  const auto path_optimizer_yaml =
    package_path + "/config/path_optimizer.param.yaml";

  rclcpp::NodeOptions node_options;

  node_options.arguments(
    {
      "--ros-args",
      "--params-file", vehicle_info_yaml,
      "--params-file", common_yaml,
      "--params-file", nearest_search_yaml,
      "--params-file", path_optimizer_yaml
    });

  node_options.parameter_overrides(
    {
      rclcpp::Parameter("mpt.kinematics.model_type", "bicycle"),
      rclcpp::Parameter("mpt.weight.steer_input_weight", 3.0),
      rclcpp::Parameter("mpt.weight.steer_rate_weight", 2.0),
    });

  auto node =
    std::make_shared<rclcpp::Node>("mpt_optimizer_test_2ws", node_options);

  const auto vehicle_info =
    autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo();

  const auto ego_nearest_param =
    EgoNearestParam(node.get());

  const auto traj_param =
    TrajectoryParam(node.get());

  auto debug_data_ptr =
    std::make_shared<DebugData>();

  auto time_keeper =
    std::make_shared<autoware_utils::TimeKeeper>();

  MPTOptimizer optimizer(
    node.get(),
    false,
    ego_nearest_param,
    vehicle_info,
    traj_param,
    debug_data_ptr,
    time_keeper);

  std::vector<ReferencePoint> ref_points(N_ref);

  for (auto & ref_point : ref_points) {
    ref_point.normalized_avoidance_cost = 0.0;
  }

  std::vector<TrajectoryPoint> traj_points(N_ref);

  const auto value_matrix =
    optimizer.calcValueMatrix(ref_points, traj_points);

  ASSERT_EQ(value_matrix.R.rows(), N_u);
  ASSERT_EQ(value_matrix.R.cols(), N_u);

  const double steer_input_weight =
    optimizer.mpt_param_.steer_input_weight;

  const double steer_rate_weight =
    optimizer.mpt_param_.steer_rate_weight;

  // Diagonal terms.
  EXPECT_DOUBLE_EQ(
    value_matrix.R.coeff(0, 0),
    steer_input_weight + steer_rate_weight);

  EXPECT_DOUBLE_EQ(
    value_matrix.R.coeff(1, 1),
    steer_input_weight + steer_rate_weight);

  // Steering rate:
  // (delta_1 - delta_0)^2
  EXPECT_DOUBLE_EQ(
    value_matrix.R.coeff(0, 1),
    -steer_rate_weight);

  EXPECT_DOUBLE_EQ(
    value_matrix.R.coeff(1, 0),
    -steer_rate_weight);
}

TEST_F(MPTOptimizerTest, SteeringConstraintBounds4WS)
{
  constexpr size_t N_ref = 3;
  constexpr size_t D_x = 2;
  constexpr size_t D_u = 2;
  constexpr size_t N_x = N_ref * D_x;
  constexpr size_t N_u = (N_ref - 1) * D_u;

  // Create a dedicated optimizer with only the steering constraint enabled.
  const auto package_path =
    ament_index_cpp::get_package_share_directory("autoware_path_optimizer");

  const auto test_utils_package =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");

  const auto vehicle_info_yaml =
    test_utils_package + "/config/test_vehicle_info.param.yaml";

  const auto common_yaml =
    test_utils_package + "/config/test_common.param.yaml";

  const auto nearest_search_yaml =
    test_utils_package + "/config/test_nearest_search.param.yaml";

  const auto path_optimizer_yaml =
    package_path + "/config/path_optimizer.param.yaml";

  rclcpp::NodeOptions node_options;

  node_options.arguments(
    {
      "--ros-args",
      "--params-file", vehicle_info_yaml,
      "--params-file", common_yaml,
      "--params-file", nearest_search_yaml,
      "--params-file", path_optimizer_yaml
    });

  node_options.parameter_overrides(
    {
      rclcpp::Parameter("mpt.kinematics.model_type", "4ws"),
      rclcpp::Parameter("mpt.constraint.soft_constraint", false),
      rclcpp::Parameter("mpt.constraint.hard_constraint", false),
    });

  auto node =
    std::make_shared<rclcpp::Node>("mpt_optimizer_test_steering_constraint", node_options);

  const auto vehicle_info =
    autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo();

  const auto ego_nearest_param =
    EgoNearestParam(node.get());

  const auto traj_param =
    TrajectoryParam(node.get());

  auto debug_data_ptr =
    std::make_shared<DebugData>();

  auto time_keeper =
    std::make_shared<autoware_utils::TimeKeeper>();

  MPTOptimizer optimizer(
    node.get(),
    false,
    ego_nearest_param,
    vehicle_info,
    traj_param,
    debug_data_ptr,
    time_keeper);

  std::vector<ReferencePoint> ref_points(N_ref);

  const size_t N_collision_check =
    optimizer.vehicle_circle_longitudinal_offsets_.size();

  for (auto & ref_point : ref_points) {
    ref_point.curvature = 0.1;

    // Data required by the collision-free section.
    ref_point.beta.resize(N_collision_check, 0.0);

    ref_point.bounds_on_constraints.resize(
      N_collision_check,
      Bounds{-10.0, 10.0});
  }

  StateEquationGenerator::Matrix mpt_mat;

  mpt_mat.A = Eigen::MatrixXd::Zero(N_x, N_x);
  mpt_mat.B = Eigen::MatrixXd::Zero(N_x, N_u);
  mpt_mat.W = Eigen::VectorXd::Zero(N_x);

  const auto constraint_matrix =
    optimizer.calcConstraintMatrix(mpt_mat, ref_points);

  const double max_steer =
    optimizer.mpt_param_.max_steer_rad;

  // The steering constraints are the last N_u rows.
  const size_t steer_row_start =
    constraint_matrix.linear.rows() - N_u;

  for (size_t i = 0; i < N_u; ++i) {
    // Check physical steering limits.
    EXPECT_DOUBLE_EQ(
      constraint_matrix.lower_bound(steer_row_start + i),
      -max_steer);

    EXPECT_DOUBLE_EQ(
      constraint_matrix.upper_bound(steer_row_start + i),
      max_steer);

    // Check that each steering constraint acts on the
    // corresponding control variable.
    for (size_t j = 0; j < N_u; ++j) {
      const double expected = (i == j) ? 1.0 : 0.0;

      EXPECT_DOUBLE_EQ(
        constraint_matrix.linear(steer_row_start + i, N_x + j),
        expected);
    }
  }
}

}  // namespace autoware::path_optimizer