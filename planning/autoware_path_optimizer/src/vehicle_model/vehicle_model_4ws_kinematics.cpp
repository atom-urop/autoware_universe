#include "autoware/path_optimizer/vehicle_model/vehicle_model_4ws_kinematics.hpp"
#include "autoware/interpolation/linear_interpolation.hpp"

#include <algorithm>
#include <cmath>

Kinematics4WSModel::Kinematics4WSModel(
  const double wheelbase,
  const double steer_limit,
  const std::vector<double> & k_ref,
  const std::vector<double> & delta_f_ref,
  const std::vector<double> & rr_ref)
: VehicleModelInterface(2, 2, 2, wheelbase, steer_limit),
  k_ref_(k_ref),
  delta_f_ref_(delta_f_ref),
  rr_ref_(rr_ref)
{
}

void Kinematics4WSModel::calculateStateEquationMatrix(
  Eigen::MatrixXd & Ad,
  Eigen::MatrixXd & Bd,
  Eigen::MatrixXd & Wd,
  const double curvature,
  const double ds) const
{
  const double abs_curvature =
    std::min(std::abs(curvature), k_ref_.back());

  const double delta_f_abs =
    autoware::interpolation::lerp(
      k_ref_,
      delta_f_ref_,
      abs_curvature);

  const double rr =
    autoware::interpolation::lerp(
      k_ref_,
      rr_ref_,
      abs_curvature);

  const double delta_f_ref =
    std::copysign(delta_f_abs, curvature);

  const double delta_r_ref =
    rr * delta_f_ref;

  const double kappa_model =
    std::sin(delta_f_ref - delta_r_ref) /
    (wheelbase_ * std::cos(delta_f_ref));

  const double J_f =
    std::cos(delta_r_ref) /
    (wheelbase_ * std::pow(std::cos(delta_f_ref), 2.0));

  const double J_r =
    -std::cos(delta_f_ref - delta_r_ref) /
    (wheelbase_ * std::cos(delta_f_ref));

  Ad << 1.0, ds,
        0.0, 1.0;

  Bd << 0.0, 0.0,
        ds * J_f, ds * J_r;

  Wd << 0.0,
        ds * (
          kappa_model
          - J_f * delta_f_ref
          - J_r * delta_r_ref
          - curvature);
}