// Copyright 2026 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/stanley_lateral_controller/stanley.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include "autoware/stanley_lateral_controller/stanley_utils.hpp"

#include <fmt/format.h>


namespace autoware::motion::control::stanley_lateral_controller
{

Stanley::Stanley(
  const double traj_resample_dist,
  const double curvature_calculation_distance,
  const double wheel_base,
  const double max_steer_angle,
  const double k_gain1,
  const double k_soft,
  const double k_gain2,
  const std::vector<double> & k_ref_LUT,
  const std::vector<double> & rr_LUT,
  const std::vector<double> & kappa_gain_LUT,
  const std::vector<double> & gain_4WS_LUT)
: m_traj_resample_dist(traj_resample_dist),
  m_curvature_calculation_distance(curvature_calculation_distance),
  m_wheel_base(wheel_base),
  m_max_steer_angle(max_steer_angle),
  m_k_gain1(k_gain1),
  m_k_soft(k_soft),
  m_k_gain2(k_gain2),
  m_k_ref_LUT(k_ref_LUT),
  m_rr_LUT(rr_LUT),
  m_kappa_gain_LUT(kappa_gain_LUT),
  m_gain_4WS_LUT(gain_4WS_LUT)
{
}

ResultWithReason Stanley::calculateStanley(
  const Trajectory & reference_trajectory,
  const Odometry & current_front_odometry,
  const Odometry & predicted_front_odometry,
  Lateral & ctrl_cmd,
  double & rear_steer)
{
  StanleyData stanley_data;

  const auto data_result = getData(
    reference_trajectory,
    current_front_odometry,
    predicted_front_odometry,
    stanley_data);

  if (!data_result.result) {
    return ResultWithReason{
      false,
      fmt::format("getting Stanley Data ({}).", data_result.reason)};
  }

  return calculateControl(
    stanley_data,
    ctrl_cmd,
    rear_steer);

}

ResultWithReason Stanley::getData(
  const Trajectory & reference_trajectory,
  const Odometry & current_front_odometry,
  const Odometry & predicted_front_odometry,
  StanleyData & data)
{
  size_t current_nearest_idx;

  const auto current_nearest_pose = calcNearestPoseInterpStanley(
    reference_trajectory,
    current_front_odometry.pose.pose,
    ego_nearest_dist_threshold,
    ego_nearest_yaw_threshold,
    current_nearest_idx);

  size_t predicted_nearest_idx;

  const auto predicted_nearest_pose = calcNearestPoseInterpStanley(
    reference_trajectory,
    predicted_front_odometry.pose.pose,
    ego_nearest_dist_threshold,
    ego_nearest_yaw_threshold,
    predicted_nearest_idx);

  const double predicted_lateral_error = calcLateralErrorStanley(
    predicted_front_odometry,
    predicted_nearest_pose);

  data.lateral_error = predicted_lateral_error;

  // Calculate the curvature vector for the entire reference trajectory.
  const auto curvature_vector = calcCurvatureVectorStanley(
    reference_trajectory,
    m_traj_resample_dist,
    m_curvature_calculation_distance);

  const double predicted_reference_curvature =
    curvature_vector.at(predicted_nearest_idx);

  const double current_reference_curvature =
    curvature_vector.at(current_nearest_idx);

  const double reference_curvature =
  std::abs(predicted_reference_curvature) >= std::abs(current_reference_curvature)
    ? predicted_reference_curvature
    : current_reference_curvature;

  const double curvature_max =
    std::sin(2.0 * m_max_steer_angle) /
    (m_wheel_base * std::cos(-m_max_steer_angle));

  data.reference_curvature =
    std::copysign(
      std::min(std::abs(reference_curvature), curvature_max),
      reference_curvature);

  data.longitudinal_velocity = 
    predicted_front_odometry.twist.twist.linear.x;

  (void)current_nearest_pose;

  return ResultWithReason{true};
}

ResultWithReason Stanley::calculateControl(
  const StanleyData & stanley_data,
  Lateral & ctrl_cmd,
  double & rear_steer)
{
  // ============================================================
  // 1. 4WS parameters from LUTs
  // ============================================================

  const double rr = calculateRearSteeringRatio(
    stanley_data.reference_curvature,
    m_k_ref_LUT,
    m_rr_LUT);

  const double gain_4WS = calculate4WSGain(
    stanley_data.reference_curvature,
    m_kappa_gain_LUT,
    m_gain_4WS_LUT);

  // ============================================================
  // 2. Stanley cross-track correction
  // ============================================================

  const double cross_track_term = std::atan2(
    m_k_gain1 * stanley_data.lateral_error,
    stanley_data.longitudinal_velocity + m_k_soft);

  // ============================================================
  // 3. Equivalent 2WS front steering
  // ============================================================

  double front_steer =
    m_k_gain2 * cross_track_term;

  // ============================================================
  // 4. Saturate equivalent 2WS steering
  // ============================================================

  front_steer = std::clamp(
    front_steer,
    -m_max_steer_angle,
    m_max_steer_angle);

  // ============================================================
  // 5. Apply 4WS gain
  // ============================================================

  front_steer =
    gain_4WS * front_steer;

  // ============================================================
  // 6. Rear steering
  // ============================================================

  rear_steer =
    rr * front_steer;

  // ============================================================
  // 7. Front steering output
  // ============================================================

  ctrl_cmd.steering_tire_angle =
    front_steer;

  return ResultWithReason{true, ""};
}

}  // namespace autoware::motion::control::stanley_lateral_controller