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

#ifndef AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_HPP_
#define AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_HPP_

#include "autoware_control_msgs/msg/lateral.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <vector>

#include <string>

namespace autoware::motion::control::stanley_lateral_controller
{

using autoware_control_msgs::msg::Lateral;
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

struct ResultWithReason
{
  bool result{false};
  std::string reason{""};
};

struct StanleyData
{
  double lateral_error;
  double reference_curvature;
  double longitudinal_velocity;
};

struct StanleyDebugData
{
  double lateral_error;
  double reference_curvature;
  double longitudinal_velocity;

  double cross_track_term;
  double front_steer_2ws;
  double front_steer;
  double rear_steer;
  double rr;
  double gain_4ws;
};

class Stanley
{

private:
  double m_traj_resample_dist;
  double m_curvature_calculation_distance;
  double m_wheel_base;
  double m_max_steer_angle;
  double m_k_gain1;
  double m_k_soft;
  double m_k_gain2;
  std::vector<double> m_k_ref_LUT;
  std::vector<double> m_rr_LUT;
  std::vector<double> m_kappa_gain_LUT;
  std::vector<double> m_gain_4WS_LUT;

public:
  explicit Stanley(
    double traj_resample_dist,
    double curvature_calculation_distance,
    double wheel_base,
    double max_steer_angle,
    double k_gain1,
    double k_soft,
    double k_gain2,
    const std::vector<double> & k_ref_LUT,
    const std::vector<double> & rr_LUT,
    const std::vector<double> & kappa_gain_LUT,
    const std::vector<double> & gain_4WS_LUT);

  ResultWithReason calculateStanley(
    const Trajectory & reference_trajectory,
    const Odometry & current_front_odometry,
    const Odometry & predicted_front_odometry,
    Lateral & ctrl_cmd,
    double & rear_steer,
    StanleyDebugData & stanley_debug_data);

  ResultWithReason getData(
    const Trajectory & reference_trajectory,
    const Odometry & current_front_odometry,
    const Odometry & predicted_front_odometry,
    StanleyData & data);
  
  ResultWithReason calculateControl(
    const StanleyData & stanley_data,
    Lateral & ctrl_cmd,
    double & rear_steer,
    StanleyDebugData & stanley_debug_data);

  double ego_nearest_dist_threshold{3.0};
  double ego_nearest_yaw_threshold{1.5707963267948966};
};

}  // namespace autoware::motion::control::stanley_lateral_controller

#endif  // AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_HPP_