// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__PATH_OPTIMIZER__VEHICLE_MODEL__VEHICLE_MODEL_4WS_KINEMATICS_HPP_
#define AUTOWARE__PATH_OPTIMIZER__VEHICLE_MODEL__VEHICLE_MODEL_4WS_KINEMATICS_HPP_

#include "autoware/path_optimizer/vehicle_model/vehicle_model_interface.hpp"

#include <Eigen/Core>
#include <Eigen/LU>

#include <vector>

class Kinematics4WSModel : public VehicleModelInterface
{
public:
  Kinematics4WSModel(
    const double wheelbase,
    const double steer_limit,
    const std::vector<double> & k_ref,
    const std::vector<double> & delta_f_ref,
    const std::vector<double> & rr_ref);

  virtual ~Kinematics4WSModel() = default;

  void calculateStateEquationMatrix(
    Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Wd, const double curvature,
    const double ds) const override;

private:
  std::vector<double> k_ref_;
  std::vector<double> delta_f_ref_;
  std::vector<double> rr_ref_;
};

#endif  // AUTOWARE__PATH_OPTIMIZER__VEHICLE_MODEL__VEHICLE_MODEL_4WS_KINEMATICS_HPP_