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

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>

namespace autoware::motion::control::stanley_lateral_controller
{
StanleyLateralController::StanleyLateralController(rclcpp::Node & node)
{
  m_traj_resample_dist =
    node.declare_parameter<double>("traj_resample_dist");

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

  m_k_gain1 =
    node.declare_parameter<double>("k_gain1");

  m_k_soft =
    node.declare_parameter<double>("k_soft");

  m_k_gain2 =
    node.declare_parameter<double>("k_gain2");
}

}  // namespace autoware::motion::control::stanley_lateral_controller