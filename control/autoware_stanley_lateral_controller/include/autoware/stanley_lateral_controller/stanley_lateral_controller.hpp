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

#ifndef AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_LATERAL_CONTROLLER_HPP_
#define AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_LATERAL_CONTROLLER_HPP_

#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"
#include "autoware/stanley_lateral_controller/stanley.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_control_msgs/msg/lateral.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace autoware::motion::control::stanley_lateral_controller
{

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

using autoware_control_msgs::msg::Lateral;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::SteeringReport;
using nav_msgs::msg::Odometry;
using trajectory_follower::LateralHorizon;

class StanleyLateralController : public trajectory_follower::LateralControllerBase
{
public:
  explicit StanleyLateralController(rclcpp::Node & node);
  ~StanleyLateralController() = default;

private:

  // -- current vehicle state --
  Odometry m_current_kinematic_state;
  SteeringReport m_current_steering;
  Trajectory m_current_trajectory;

  // -- system --
  double m_wheel_base;
  double m_max_steer_angle;
  double m_tau_max;
  double m_d0;

  double m_ego_nearest_dist_threshold;
  double m_ego_nearest_yaw_threshold;

  // -- Stanley algorithm --
  std::shared_ptr<Stanley> m_stanley;

  // -- steer offset --
  bool m_enable_auto_steering_offset_removal;
  double m_update_vel_threshold;
  double m_update_steer_threshold;
  int m_average_num;
  double m_steering_offset_limit;

  void setTrajectory(const Trajectory & msg);

  [[nodiscard]] bool isValidTrajectory(const Trajectory & traj) const;

  bool isReady(const trajectory_follower::InputData & input_data) override;

  trajectory_follower::LateralOutput run(
    trajectory_follower::InputData const & input_data) override;
};

}  // namespace autoware::motion::control::stanley_lateral_controller

#endif  // AUTOWARE__STANLEY_LATERAL_CONTROLLER__STANLEY_LATERAL_CONTROLLER_HPP_