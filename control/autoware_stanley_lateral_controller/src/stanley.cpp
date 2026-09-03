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

namespace autoware::motion::control::stanley_lateral_controller
{

ResultWithReason Stanley::calculateStanley(
  const Trajectory & reference_trajectory,
  const Odometry & current_front_odometry,
  const Odometry & predicted_front_odometry,
  Lateral & ctrl_cmd,
  double & rear_steer)
{
  return ResultWithReason{true};
}

}  // namespace autoware::motion::control::stanley_lateral_controller