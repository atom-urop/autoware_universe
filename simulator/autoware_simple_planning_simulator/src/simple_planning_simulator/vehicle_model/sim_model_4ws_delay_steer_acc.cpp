// Copyright 2025 The Autoware Foundation.
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

#include "autoware/simple_planning_simulator/vehicle_model/sim_model_4ws_delay_steer_acc.hpp"

#include <algorithm>

namespace autoware::simulator::simple_planning_simulator
{

SimModel4wsDelaySteerAcc::SimModel4wsDelaySteerAcc(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double acc_delay, double acc_time_constant, double steer_delay,
  double steer_time_constant, double steer_dead_band, double steer_bias,
  double debug_acc_scaling_factor, double debug_steer_scaling_factor)
: SimModelInterface(7 /* dim x */, 3 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  acc_delay_(acc_delay),
  acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_dead_band_(steer_dead_band),
  steer_bias_(steer_bias),
  debug_acc_scaling_factor_(std::max(debug_acc_scaling_factor, 0.0)),
  debug_steer_scaling_factor_(std::max(debug_steer_scaling_factor, 0.0))
{
  initializeInputQueue(dt);
}

double SimModel4wsDelaySteerAcc::getX()
{
  return state_(IDX::X);
}
double SimModel4wsDelaySteerAcc::getY()
{
  return state_(IDX::Y);
}
double SimModel4wsDelaySteerAcc::getYaw()
{
  return state_(IDX::YAW);
}
double SimModel4wsDelaySteerAcc::getVx()
{
  return state_(IDX::VX);
}
double SimModel4wsDelaySteerAcc::getVy()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER_REAR));//deducing the Vy
}

double SimModel4wsDelaySteerAcc::getAx()
{
  return state_(IDX::ACCX);
}
double SimModel4wsDelaySteerAcc::getWz()
{
    return state_(IDX::VX) *
         (std::tan(state_(IDX::STEER_FRONT)) - std::tan(state_(IDX::STEER_REAR))) / wheelbase_; //for the 4WS vehicle model
}
double SimModel4wsDelaySteerAcc::getSteer()
{
  return state_(IDX::STEER_FRONT) + steer_bias_;
}

double SimModel4wsDelaySteerAcc::getSteerRear()
{
  return state_(IDX::STEER_REAR);
}

void SimModel4wsDelaySteerAcc::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  acc_input_queue_.push_back(input_(IDX_U::ACCX_DES));
  delayed_input(IDX_U::ACCX_DES) = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  steer_input_queue_.push_back(input_(IDX_U::STEER_FRONT_DES));
  delayed_input(IDX_U::STEER_FRONT_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();

  // --Adding for the 4WS vehicle model---
  steer_rear_input_queue_.push_back(input_(IDX_U::STEER_REAR_DES));
  delayed_input(IDX_U::STEER_REAR_DES) = steer_rear_input_queue_.front();
  steer_rear_input_queue_.pop_front();

  //Adding for the 4Ws vehicle model
    const auto prev_state = state_;
  updateRungeKutta(dt, delayed_input);

  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));
    updateStateWithGear(state_, prev_state, gear_, dt);//Added for the 4Ws vehicle model
}

void SimModel4wsDelaySteerAcc::initializeInputQueue(const double & dt)
{
  size_t acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  acc_input_queue_.resize(acc_input_queue_size);
  std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), 0.0);

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);
  // ---Adding for the 4WS model---

  size_t steer_rear_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_rear_input_queue_.resize(steer_rear_input_queue_size);
  std::fill(steer_rear_input_queue_.begin(), steer_rear_input_queue_.end(), 0.0);
}

Eigen::VectorXd SimModel4wsDelaySteerAcc::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  const double vel = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double acc = sat(state(IDX::ACCX), vx_rate_lim_, -vx_rate_lim_);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER_FRONT);
  const double steer_rear = state(IDX::STEER_REAR);
  const double acc_des =
    sat(input(IDX_U::ACCX_DES), vx_rate_lim_, -vx_rate_lim_) * debug_acc_scaling_factor_;
  const double steer_des =
    sat(input(IDX_U::STEER_FRONT_DES), steer_lim_, -steer_lim_) * debug_steer_scaling_factor_;
  const double steer_diff = getSteer() - steer_des;
  const double steer_diff_with_dead_band = std::invoke([&]() {
    if (steer_diff > steer_dead_band_) {
      return steer_diff - steer_dead_band_;
    } else if (steer_diff < -steer_dead_band_) {
      return steer_diff + steer_dead_band_;
    } else {
      return 0.0;
    }
  });
  const double steer_rate =
    sat(-steer_diff_with_dead_band / steer_time_constant_, steer_rate_lim_, -steer_rate_lim_);

    // ---- rear axle: same actuator model as the front ----
  const double steer_rear_des =
    sat(input(IDX_U::STEER_REAR_DES), steer_lim_, -steer_lim_) * debug_steer_scaling_factor_;
  const double steer_rear_diff = getSteerRear() - steer_rear_des;
  const double steer_rear_diff_with_dead_band = std::invoke([&]() {
    if (steer_rear_diff > steer_dead_band_) {
      return steer_rear_diff - steer_dead_band_;
    } else if (steer_rear_diff < -steer_dead_band_) {
      return steer_rear_diff + steer_dead_band_;
    } else {
      return 0.0;
    }
  });
  const double steer_rate_rear = sat(
    -steer_rear_diff_with_dead_band / steer_time_constant_, steer_rate_lim_, -steer_rate_lim_);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  const double vy = vel * std::tan(steer_rear);
  d_state(IDX::X) = vel * cos(yaw) - vy * sin(yaw);
  d_state(IDX::Y) = vel * sin(yaw) + vy * cos(yaw);
  d_state(IDX::YAW) = vel * (std::tan(steer) - std::tan(steer_rear)) / wheelbase_;
  d_state(IDX::VX) = acc;
  d_state(IDX::STEER_FRONT) = steer_rate;
  d_state(IDX::STEER_REAR) = steer_rate_rear;//added for the 4WS steering vehicle model
  d_state(IDX::ACCX) = -(acc - acc_des) / acc_time_constant_;

  return d_state;
}
//Adding a function for the 4Ws vehicle model
void SimModel4wsDelaySteerAcc::updateStateWithGear(
  Eigen::VectorXd & state, const Eigen::VectorXd & prev_state, const uint8_t gear, const double dt)
{
  const auto setStopState = [&]() {
    state(IDX::VX) = 0.0;
    state(IDX::X) = prev_state(IDX::X);
    state(IDX::Y) = prev_state(IDX::Y);
    state(IDX::YAW) = prev_state(IDX::YAW);
    state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
  };

  using autoware_vehicle_msgs::msg::GearCommand;
  if (
    gear == GearCommand::DRIVE || gear == GearCommand::DRIVE_2 || gear == GearCommand::DRIVE_3 ||
    gear == GearCommand::DRIVE_4 || gear == GearCommand::DRIVE_5 || gear == GearCommand::DRIVE_6 ||
    gear == GearCommand::DRIVE_7 || gear == GearCommand::DRIVE_8 || gear == GearCommand::DRIVE_9 ||
    gear == GearCommand::DRIVE_10 || gear == GearCommand::DRIVE_11 ||
    gear == GearCommand::DRIVE_12 || gear == GearCommand::DRIVE_13 ||
    gear == GearCommand::DRIVE_14 || gear == GearCommand::DRIVE_15 ||
    gear == GearCommand::DRIVE_16 || gear == GearCommand::DRIVE_17 ||
    gear == GearCommand::DRIVE_18 || gear == GearCommand::LOW || gear == GearCommand::LOW_2) {
    if (state(IDX::VX) < 0.0) {
      setStopState();
    }
  } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
    if (state(IDX::VX) > 0.0) {
      setStopState();
    }
  } else {  // including 'gear == GearCommand::PARK'
    setStopState();
  }
}
}  // namespace autoware::simulator::simple_planning_simulator
