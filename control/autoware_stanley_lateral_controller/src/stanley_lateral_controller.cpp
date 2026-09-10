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

#include "autoware/stanley_lateral_controller/stanley_lateral_controller.hpp"

#include "autoware/stanley_lateral_controller/stanley_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>

namespace autoware::motion::control::stanley_lateral_controller
{

StanleyLateralController::StanleyLateralController(rclcpp::Node & node)
{

  const double traj_resample_dist =
    node.declare_parameter<double>("traj_resample_dist");

  const double curvature_calculation_distance =
    node.declare_parameter<double>("curvature_calculation_distance");

  m_wheel_base =
    node.declare_parameter<double>("wheel_base", 2);

  m_max_steer_angle =
  node.declare_parameter<double>("max_steer_angle", 0.7);

  const auto k_ref_LUT =
    node.declare_parameter<std::vector<double>>(
      "k_ref_LUT", std::vector<double>{});

  const auto rr_LUT =
    node.declare_parameter<std::vector<double>>(
      "rr_LUT", std::vector<double>{});
  
  const auto kappa_gain_LUT =
    node.declare_parameter<std::vector<double>>(
      "kappa_gain_LUT", std::vector<double>{});

  const auto gain_4WS_LUT =
    node.declare_parameter<std::vector<double>>(
      "gain_4WS_LUT", std::vector<double>{});

  const auto k_gain1 =
    node.declare_parameter<double>("k_gain1", 0.8);

  const auto k_soft =
    node.declare_parameter<double>("k_soft", 1.5);

  const auto k_gain2 =
    node.declare_parameter<double>("k_gain2", 15.0);

  m_stanley = std::make_unique<Stanley>(
    traj_resample_dist,
    curvature_calculation_distance,
    m_wheel_base,
    m_max_steer_angle,
    k_gain1,
    k_soft,
    k_gain2,
    k_ref_LUT,
    rr_LUT,
    kappa_gain_LUT,
    gain_4WS_LUT);

  m_tau_max =
    node.declare_parameter<double>("tau_max");

  m_d0 =
    node.declare_parameter<double>("d0");

  m_ego_nearest_dist_threshold =
  node.has_parameter("ego_nearest_dist_threshold")
    ? node.get_parameter("ego_nearest_dist_threshold").as_double()
    : 3.0;

  m_ego_nearest_yaw_threshold =
  node.has_parameter("ego_nearest_yaw_threshold")
    ? node.get_parameter("ego_nearest_yaw_threshold").as_double()
    : M_PI_2;

  m_stanley->ego_nearest_dist_threshold = m_ego_nearest_dist_threshold;
  m_stanley->ego_nearest_yaw_threshold = m_ego_nearest_yaw_threshold;

  m_enable_auto_steering_offset_removal =
    node.declare_parameter<bool>("enable_auto_steering_offset_removal");

  m_update_vel_threshold =
    node.declare_parameter<double>("update_vel_threshold");

  m_update_steer_threshold =
    node.declare_parameter<double>("update_steer_threshold");

  m_average_num =
    node.declare_parameter<int>("average_num");

  m_steering_offset_limit =
    node.declare_parameter<double>("steering_offset_limit");
    
}

void StanleyLateralController::setTrajectory(const Trajectory & msg)
{
  m_current_trajectory = msg;
}

bool StanleyLateralController::isValidTrajectory(const Trajectory & traj) const
{
  for (const auto & p : traj.points) {
    if (
      !std::isfinite(p.pose.position.x) ||
      !std::isfinite(p.pose.position.y) ||
      !std::isfinite(p.pose.position.z) ||
      !std::isfinite(p.pose.orientation.w) ||
      !std::isfinite(p.pose.orientation.x) ||
      !std::isfinite(p.pose.orientation.y) ||
      !std::isfinite(p.pose.orientation.z) ||
      !std::isfinite(p.longitudinal_velocity_mps) ||
      !std::isfinite(p.lateral_velocity_mps) ||
      !std::isfinite(p.heading_rate_rps) ||
      !std::isfinite(p.front_wheel_angle_rad) ||
      !std::isfinite(p.rear_wheel_angle_rad)) {
      return false;
    }
  }

  return true;
}

bool StanleyLateralController::isReady(
  const trajectory_follower::InputData & input_data)
{
  m_current_kinematic_state = input_data.current_odometry;
  m_current_steering = input_data.current_steering;

  setTrajectory(input_data.current_trajectory);

  if (m_current_trajectory.points.size() < 3) {
    return false;
  }

  if (!isValidTrajectory(m_current_trajectory)) {
    return false;
  }

  return true;
}



trajectory_follower::LateralOutput StanleyLateralController::run(
  trajectory_follower::InputData const & input_data)
{
  // Set current input data
  setTrajectory(input_data.current_trajectory);

  m_current_kinematic_state = input_data.current_odometry;
  m_current_steering = input_data.current_steering;

// Current front axle odometry
const auto current_front_odometry = rearToFrontOdometry(
  m_current_kinematic_state,
  m_wheel_base);

// Predicted front axle odometry
const auto predicted_front_odometry = rearToFrontOdometryPred(
  m_current_kinematic_state,
  m_wheel_base,
  m_tau_max,
  m_d0);

trajectory_follower::LateralOutput output;

double rear_steer = 0.0;

const auto stanley_result = m_stanley->calculateStanley(
  m_current_trajectory,
  current_front_odometry,
  predicted_front_odometry,
  output.control_cmd,
  rear_steer);

if (!stanley_result.result) {
  RCLCPP_WARN(
    rclcpp::get_logger("stanley_lateral_controller"),
    "Stanley calculation failed: %s",
    stanley_result.reason.c_str());
  return output;
}

return output;
}

}  // namespace autoware::motion::control::stanley_lateral_controller