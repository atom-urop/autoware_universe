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
};

class Stanley
{

private:
  double m_traj_resample_dist;
  double m_curvature_calculation_distance;
  double m_wheel_base;
  double m_max_steer_angle;

public:
  explicit Stanley(
    double traj_resample_dist,
    double curvature_calculation_distance,
    double wheel_base,
    double max_steer_angle);

  ResultWithReason calculateStanley(
    const Trajectory & reference_trajectory,
    const Odometry & current_front_odometry,
    const Odometry & predicted_front_odometry,
    Lateral & ctrl_cmd,
    double & rear_steer);

  ResultWithReason getData(
    const Trajectory & reference_trajectory,
    const Odometry & current_front_odometry,
    const Odometry & predicted_front_odometry,
    StanleyData & data);

  double ego_nearest_dist_threshold{3.0};
  double ego_nearest_yaw_threshold{1.5707963267948966};
};

}  // namespace autoware::motion::control::stanley_lateral_controller

#endif  // AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_HPP_