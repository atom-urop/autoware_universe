#include "autoware/path_optimizer/vehicle_model/vehicle_model_4ws_kinematics.hpp"
#include "autoware/interpolation/linear_interpolation.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

namespace
{

std::tuple<
  std::vector<double>,
  std::vector<double>,
  std::vector<double>>
loadKinematics4WSLUT()
{
  const auto package_path =
    ament_index_cpp::get_package_share_directory(
      "autoware_path_optimizer");

  const auto yaml_path =
    package_path + "/config/path_optimizer.param.yaml";

  const auto config = YAML::LoadFile(yaml_path);

  const auto parameters = config["/**"]["ros__parameters"];
  const auto kinematics = parameters["mpt"]["kinematics"];

  const auto k_ref =
    kinematics["k_ref"].as<std::vector<double>>();

  const auto delta_f_ref =
    kinematics["delta_f_ref"].as<std::vector<double>>();

  const auto rr_ref =
    kinematics["rr_ref"].as<std::vector<double>>();

  return {k_ref, delta_f_ref, rr_ref};
}

}  // namespace

TEST(Kinematics4WSModelTest, CalculateStateEquationMatrix)
{
  const auto [k_ref, delta_f_ref, rr_ref] =
    loadKinematics4WSLUT();

  ASSERT_FALSE(k_ref.empty());
  ASSERT_EQ(k_ref.size(), delta_f_ref.size());
  ASSERT_EQ(k_ref.size(), rr_ref.size());

  for (size_t i = 1; i < k_ref.size(); ++i) {
    EXPECT_GT(k_ref.at(i), k_ref.at(i - 1));
  }

  constexpr double wheelbase = 2.0;
  constexpr double steer_limit = 0.7;
  constexpr double ds = 0.5;

  Kinematics4WSModel model(
    wheelbase,
    steer_limit,
    k_ref,
    delta_f_ref,
    rr_ref);

  const size_t index = 100;

  ASSERT_LT(index, k_ref.size());

  const double curvature = k_ref.at(index);

  Eigen::MatrixXd Ad(2, 2);
  Eigen::MatrixXd Bd(2, 2);
  Eigen::MatrixXd Wd(2, 1);

  model.calculateStateEquationMatrix(
    Ad,
    Bd,
    Wd,
    curvature,
    ds);

  const double delta_f_ref_test =
    delta_f_ref.at(index);

  const double rr =
    rr_ref.at(index);

  const double delta_r_ref =
    rr * delta_f_ref_test;

  const double kappa_model =
    std::sin(delta_f_ref_test - delta_r_ref) /
    (wheelbase * std::cos(delta_f_ref_test));

  const double J_f =
    std::cos(delta_r_ref) /
    (wheelbase * std::pow(std::cos(delta_f_ref_test), 2.0));

  const double J_r =
    -std::cos(delta_f_ref_test - delta_r_ref) /
    (wheelbase * std::cos(delta_f_ref_test));

  Eigen::MatrixXd Ad_expected(2, 2);

  Ad_expected <<
    1.0, ds,
    0.0, 1.0;

  Eigen::MatrixXd Bd_expected(2, 2);

  Bd_expected <<
    0.0, 0.0,
    ds * J_f, ds * J_r;

  Eigen::VectorXd Wd_expected(2);

  Wd_expected <<
    0.0,
    ds * (
      kappa_model
      - J_f * delta_f_ref_test
      - J_r * delta_r_ref
      - curvature);

  std::cout << std::endl;
  std::cout
    << "4WS kinematic model test" << std::endl
    << "index         = " << index << std::endl
    << "curvature     = " << curvature << std::endl
    << "delta_f_ref   = " << delta_f_ref_test << std::endl
    << "rr_ref        = " << rr << std::endl
    << "delta_r_ref   = " << delta_r_ref << std::endl
    << "kappa_model   = " << kappa_model << std::endl
    << "J_f           = " << J_f << std::endl
    << "J_r           = " << J_r << std::endl
    << std::endl;

  std::cout
    << "Ad expected:" << std::endl
    << Ad_expected << std::endl
    << std::endl;

  std::cout
    << "Ad output:" << std::endl
    << Ad << std::endl
    << std::endl;

  std::cout
    << "Bd expected:" << std::endl
    << Bd_expected << std::endl
    << std::endl;

  std::cout
    << "Bd output:" << std::endl
    << Bd << std::endl
    << std::endl;

  std::cout
    << "Wd expected:" << std::endl
    << Wd_expected << std::endl
    << std::endl;

  std::cout
    << "Wd output:" << std::endl
    << Wd << std::endl;

  ASSERT_EQ(Ad.rows(), 2);
  ASSERT_EQ(Ad.cols(), 2);

  ASSERT_EQ(Bd.rows(), 2);
  ASSERT_EQ(Bd.cols(), 2);

  ASSERT_EQ(Wd.rows(), 2);
  ASSERT_EQ(Wd.cols(), 1);

  constexpr double tolerance = 1e-12;

  for (int row = 0; row < 2; ++row) {
    for (int col = 0; col < 2; ++col) {
      EXPECT_NEAR(
        Ad(row, col),
        Ad_expected(row, col),
        tolerance);

      EXPECT_NEAR(
        Bd(row, col),
        Bd_expected(row, col),
        tolerance);
    }

    EXPECT_NEAR(
      Wd(row),
      Wd_expected(row),
      tolerance);
  }

  EXPECT_GT(J_f, 0.0);
  EXPECT_LT(J_r, 0.0);
}


TEST(Kinematics4WSModelTest, NegativeCurvature)
{
  const auto [k_ref, delta_f_ref, rr_ref] =
    loadKinematics4WSLUT();

  ASSERT_FALSE(k_ref.empty());
  ASSERT_EQ(k_ref.size(), delta_f_ref.size());
  ASSERT_EQ(k_ref.size(), rr_ref.size());

  constexpr double wheelbase = 2.0;
  constexpr double steer_limit = 0.7;
  constexpr double ds = 0.5;

  Kinematics4WSModel model(
    wheelbase,
    steer_limit,
    k_ref,
    delta_f_ref,
    rr_ref);

  const size_t index = 100;

  ASSERT_LT(index, k_ref.size());

  const double curvature = -k_ref.at(index)-0.123;

  Eigen::MatrixXd Ad(2, 2);
  Eigen::MatrixXd Bd(2, 2);
  Eigen::MatrixXd Wd(2, 1);

  model.calculateStateEquationMatrix(
    Ad,
    Bd,
    Wd,
    curvature,
    ds);

  const double abs_curvature =
    std::min(std::abs(curvature), k_ref.back());

  const double delta_f_abs =
    autoware::interpolation::lerp(
      k_ref,
      delta_f_ref,
      abs_curvature);

  const double rr =
    autoware::interpolation::lerp(
      k_ref,
      rr_ref,
      abs_curvature);

  const double delta_f_ref_test =
    std::copysign(delta_f_abs, curvature);

  const double delta_r_ref =
    rr * delta_f_ref_test;

  const double kappa_model =
    std::sin(delta_f_ref_test - delta_r_ref) /
    (wheelbase * std::cos(delta_f_ref_test));

  const double J_f =
    std::cos(delta_r_ref) /
    (wheelbase * std::pow(std::cos(delta_f_ref_test), 2.0));

  const double J_r =
    -std::cos(delta_f_ref_test - delta_r_ref) /
    (wheelbase * std::cos(delta_f_ref_test));

  Eigen::MatrixXd Ad_expected(2, 2);

  Ad_expected <<
    1.0, ds,
    0.0, 1.0;

  Eigen::MatrixXd Bd_expected(2, 2);

  Bd_expected <<
    0.0, 0.0,
    ds * J_f, ds * J_r;

  Eigen::VectorXd Wd_expected(2);

  Wd_expected <<
    0.0,
    ds * (
      kappa_model
      - J_f * delta_f_ref_test
      - J_r * delta_r_ref
      - curvature);

  std::cout << std::endl;
  std::cout
    << "4WS negative curvature test" << std::endl
    << "index         = " << index << std::endl
    << "curvature     = " << curvature << std::endl
    << "delta_f_ref   = " << delta_f_ref_test << std::endl
    << "rr_ref        = " << rr << std::endl
    << "delta_r_ref   = " << delta_r_ref << std::endl
    << "kappa_model   = " << kappa_model << std::endl
    << "J_f           = " << J_f << std::endl
    << "J_r           = " << J_r << std::endl
    << std::endl;

  std::cout
    << "Ad output:" << std::endl
    << Ad << std::endl
    << std::endl;

  std::cout
    << "Bd output:" << std::endl
    << Bd << std::endl
    << std::endl;

  std::cout
    << "Wd output:" << std::endl
    << Wd << std::endl;

  constexpr double tolerance = 1e-12;

  for (int row = 0; row < 2; ++row) {
    for (int col = 0; col < 2; ++col) {
      EXPECT_NEAR(
        Ad(row, col),
        Ad_expected(row, col),
        tolerance);

      EXPECT_NEAR(
        Bd(row, col),
        Bd_expected(row, col),
        tolerance);
    }

    EXPECT_NEAR(
      Wd(row),
      Wd_expected(row),
      tolerance);
  }

  EXPECT_LT(delta_f_ref_test, 0.0);
  EXPECT_GT(delta_r_ref, 0.0);
  EXPECT_LT(J_r, 0.0);
}

TEST(Kinematics4WSModelTest, NegativeCurvatureBeyondLUT)
{
  const auto [k_ref, delta_f_ref, rr_ref] =
    loadKinematics4WSLUT();

  ASSERT_FALSE(k_ref.empty());
  ASSERT_EQ(k_ref.size(), delta_f_ref.size());
  ASSERT_EQ(k_ref.size(), rr_ref.size());

  constexpr double wheelbase = 2.0;
  constexpr double steer_limit = 0.7;
  constexpr double ds = 0.5;

  Kinematics4WSModel model(
    wheelbase,
    steer_limit,
    k_ref,
    delta_f_ref,
    rr_ref);

  // Deliberately outside the LUT range and not coincident with a LUT point.
  const double curvature = -(k_ref.back() + 0.123);

  Eigen::MatrixXd Ad(2, 2);
  Eigen::MatrixXd Bd(2, 2);
  Eigen::MatrixXd Wd(2, 1);

  model.calculateStateEquationMatrix(
    Ad,
    Bd,
    Wd,
    curvature,
    ds);

  // The model saturates |curvature| to the last LUT point.
  const double delta_f_abs = delta_f_ref.back();
  const double rr = rr_ref.back();

  const double delta_f_ref_test = -delta_f_abs;
  const double delta_r_ref = rr * delta_f_ref_test;

  const double kappa_model =
    std::sin(delta_f_ref_test - delta_r_ref) /
    (wheelbase * std::cos(delta_f_ref_test));

  const double J_f =
    std::cos(delta_r_ref) /
    (wheelbase * std::pow(std::cos(delta_f_ref_test), 2.0));

  const double J_r =
    -std::cos(delta_f_ref_test - delta_r_ref) /
    (wheelbase * std::cos(delta_f_ref_test));

  Eigen::MatrixXd Ad_expected(2, 2);
  Ad_expected <<
    1.0, ds,
    0.0, 1.0;

  Eigen::MatrixXd Bd_expected(2, 2);
  Bd_expected <<
    0.0, 0.0,
    ds * J_f, ds * J_r;

  Eigen::VectorXd Wd_expected(2);
  Wd_expected <<
    0.0,
    ds * (
      kappa_model
      - J_f * delta_f_ref_test
      - J_r * delta_r_ref
      - curvature);

  std::cout << "\n4WS negative curvature beyond LUT test\n";
  std::cout << "curvature          = " << curvature << "\n";
  std::cout << "k_ref.back()       = " << k_ref.back() << "\n";
  std::cout << "delta_f_ref        = " << delta_f_ref_test << "\n";
  std::cout << "rr_ref.back()      = " << rr << "\n";
  std::cout << "delta_r_ref        = " << delta_r_ref << "\n";
  std::cout << "kappa_model        = " << kappa_model << "\n";
  std::cout << "J_f                = " << J_f << "\n";
  std::cout << "J_r                = " << J_r << "\n";

  std::cout << "\nAd output:\n" << Ad << "\n";
  std::cout << "\nBd output:\n" << Bd << "\n";
  std::cout << "\nWd output:\n" << Wd << "\n";

  ASSERT_EQ(Ad.rows(), 2);
  ASSERT_EQ(Ad.cols(), 2);
  ASSERT_EQ(Bd.rows(), 2);
  ASSERT_EQ(Bd.cols(), 2);
  ASSERT_EQ(Wd.rows(), 2);
  ASSERT_EQ(Wd.cols(), 1);

  constexpr double tolerance = 1e-12;

  for (int row = 0; row < 2; ++row) {
    for (int col = 0; col < 2; ++col) {
      EXPECT_NEAR(Ad(row, col), Ad_expected(row, col), tolerance);
      EXPECT_NEAR(Bd(row, col), Bd_expected(row, col), tolerance);
    }

    EXPECT_NEAR(Wd(row), Wd_expected(row), tolerance);
  }

  // Verify the expected sign behavior.
  EXPECT_LT(delta_f_ref_test, 0.0);
  EXPECT_GT(delta_r_ref, 0.0);
  EXPECT_LT(kappa_model, 0.0);
  EXPECT_GT(J_f, 0.0);
  EXPECT_LT(J_r, 0.0);
}

TEST(Kinematics4WSModelTest, PositiveCurvatureBeyondLUT)
{
  const auto [k_ref, delta_f_ref, rr_ref] =
    loadKinematics4WSLUT();

  ASSERT_FALSE(k_ref.empty());
  ASSERT_EQ(k_ref.size(), delta_f_ref.size());
  ASSERT_EQ(k_ref.size(), rr_ref.size());

  constexpr double wheelbase = 2.0;
  constexpr double steer_limit = 0.7;
  constexpr double ds = 0.5;

  Kinematics4WSModel model(
    wheelbase,
    steer_limit,
    k_ref,
    delta_f_ref,
    rr_ref);

  // Deliberately outside the LUT range and not coincident with a LUT point.
  const double curvature = k_ref.back() + 0.123;

  Eigen::MatrixXd Ad(2, 2);
  Eigen::MatrixXd Bd(2, 2);
  Eigen::MatrixXd Wd(2, 1);

  model.calculateStateEquationMatrix(
    Ad,
    Bd,
    Wd,
    curvature,
    ds);

  // The model saturates |curvature| to the last LUT point.
  const double delta_f_abs = delta_f_ref.back();
  const double rr = rr_ref.back();

  const double delta_f_ref_test = delta_f_abs;
  const double delta_r_ref = rr * delta_f_ref_test;

  const double kappa_model =
    std::sin(delta_f_ref_test - delta_r_ref) /
    (wheelbase * std::cos(delta_f_ref_test));

  const double J_f =
    std::cos(delta_r_ref) /
    (wheelbase * std::pow(std::cos(delta_f_ref_test), 2.0));

  const double J_r =
    -std::cos(delta_f_ref_test - delta_r_ref) /
    (wheelbase * std::cos(delta_f_ref_test));

  Eigen::MatrixXd Ad_expected(2, 2);
  Ad_expected <<
    1.0, ds,
    0.0, 1.0;

  Eigen::MatrixXd Bd_expected(2, 2);
  Bd_expected <<
    0.0, 0.0,
    ds * J_f, ds * J_r;

  Eigen::VectorXd Wd_expected(2);
  Wd_expected <<
    0.0,
    ds * (
      kappa_model
      - J_f * delta_f_ref_test
      - J_r * delta_r_ref
      - curvature);

  std::cout << "\n4WS positive curvature beyond LUT test\n";
  std::cout << "curvature          = " << curvature << "\n";
  std::cout << "k_ref.back()       = " << k_ref.back() << "\n";
  std::cout << "delta_f_ref        = " << delta_f_ref_test << "\n";
  std::cout << "rr_ref.back()      = " << rr << "\n";
  std::cout << "delta_r_ref        = " << delta_r_ref << "\n";
  std::cout << "kappa_model        = " << kappa_model << "\n";
  std::cout << "J_f                = " << J_f << "\n";
  std::cout << "J_r                = " << J_r << "\n";

  std::cout << "\nAd output:\n" << Ad << "\n";
  std::cout << "\nBd output:\n" << Bd << "\n";
  std::cout << "\nWd output:\n" << Wd << "\n";

  ASSERT_EQ(Ad.rows(), 2);
  ASSERT_EQ(Ad.cols(), 2);
  ASSERT_EQ(Bd.rows(), 2);
  ASSERT_EQ(Bd.cols(), 2);
  ASSERT_EQ(Wd.rows(), 2);
  ASSERT_EQ(Wd.cols(), 1);

  constexpr double tolerance = 1e-12;

  for (int row = 0; row < 2; ++row) {
    for (int col = 0; col < 2; ++col) {
      EXPECT_NEAR(Ad(row, col), Ad_expected(row, col), tolerance);
      EXPECT_NEAR(Bd(row, col), Bd_expected(row, col), tolerance);
    }

    EXPECT_NEAR(Wd(row), Wd_expected(row), tolerance);
  }

  // Verify the expected sign behavior.
  EXPECT_GT(delta_f_ref_test, 0.0);
  EXPECT_LT(delta_r_ref, 0.0);
  EXPECT_GT(kappa_model, 0.0);
  EXPECT_GT(J_f, 0.0);
  EXPECT_LT(J_r, 0.0);
}