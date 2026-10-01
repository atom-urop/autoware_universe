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

#include "autoware/path_optimizer/utils/trajectory_utils.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/path_optimizer/mpt_optimizer.hpp"
#include "autoware/path_optimizer/utils/geometry_utils.hpp"

#include <tf2/utils.hpp>

#include "autoware/interpolation/linear_interpolation.hpp"

#include <cmath>

#include "autoware_planning_msgs/msg/path_point.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <stack>
#include <vector>

namespace autoware::path_optimizer
{

namespace trajectory_utils
{

template <>
TrajectoryPoint convertToTrajectoryPoint(const ReferencePoint & ref_point)
{
  TrajectoryPoint traj_point;
  traj_point.pose = autoware_utils::get_pose(ref_point);
  traj_point.longitudinal_velocity_mps = autoware_utils::get_longitudinal_velocity(ref_point);
  return traj_point;
}

ReferencePoint convertToReferencePoint(const TrajectoryPoint & traj_point)
{
  ReferencePoint ref_point;

  ref_point.pose = traj_point.pose;
  ref_point.longitudinal_velocity_mps = traj_point.longitudinal_velocity_mps;
  return ref_point;
}

std::vector<ReferencePoint> convertToReferencePoints(
  const std::vector<TrajectoryPoint> & traj_points)
{
  std::vector<ReferencePoint> ref_points;
  for (const auto & traj_point : traj_points) {
    const auto ref_point = convertToReferencePoint(traj_point);
    ref_points.push_back(ref_point);
  }

  return ref_points;
}

std::vector<ReferencePoint> sanitizePoints(const std::vector<ReferencePoint> & points)
{
  std::vector<ReferencePoint> output;
  for (size_t i = 0; i < points.size(); i++) {
    if (i > 0) {
      const auto & current_pos = points.at(i).pose.position;
      const auto & prev_pos = points.at(i - 1).pose.position;
      if (
        std::fabs(current_pos.x - prev_pos.x) < 1e-6 &&
        std::fabs(current_pos.y - prev_pos.y) < 1e-6) {
        continue;
      }
    }
    output.push_back(points.at(i));
  }
  return output;
}

geometry_msgs::msg::Point getNearestPosition(
  const std::vector<ReferencePoint> & points, const int target_idx, const double offset)
{
  double sum_arc_length = 0.0;
  for (size_t i = target_idx; i < points.size(); ++i) {
    sum_arc_length += points.at(i).delta_arc_length;

    if (offset < sum_arc_length) {
      return points.at(i).pose.position;
    }
  }

  return points.back().pose.position;
}

std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  constexpr bool enable_resampling_stop_point = true;

  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(
    traj, interval, false, true, true, enable_resampling_stop_point);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

// NOTE: stop point will not be resampled
std::vector<TrajectoryPoint> resampleTrajectoryPointsWithoutStopPoint(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  constexpr bool enable_resampling_stop_point = false;

  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(
    traj, interval, false, true, true, enable_resampling_stop_point);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

std::vector<ReferencePoint> resampleReferencePoints(
  const std::vector<ReferencePoint> & ref_points, const double interval)
{
  // resample pose and velocity
  const auto traj_points = convertToTrajectoryPoints(ref_points);
  const auto resampled_traj_points =
    resampleTrajectoryPointsWithoutStopPoint(traj_points, interval);
  const auto resampled_ref_points = convertToReferencePoints(resampled_traj_points);

  // resample curvature
  std::vector<double> base_keys;
  std::vector<double> base_values;
  for (size_t i = 0; i < ref_points.size(); ++i) {
    if (i == 0) {
      base_keys.push_back(0.0);
    } else {
      const double delta_arc_length =
        autoware_utils::calc_distance2d(ref_points.at(i), ref_points.at(i - 1));
      base_keys.push_back(base_keys.back() + delta_arc_length);
    }

    base_values.push_back(ref_points.at(i).curvature);
  }

  std::vector<double> query_keys;
  for (size_t i = 0; i < resampled_ref_points.size(); ++i) {
    if (i == 0) {
      query_keys.push_back(0.0);
    } else {
      const double delta_arc_length =
        autoware_utils::calc_distance2d(resampled_ref_points.at(i), resampled_ref_points.at(i - 1));
      const double key = query_keys.back() + delta_arc_length;
      if (base_keys.back() < key) {
        break;
      }

      query_keys.push_back(key);
    }
  }

  if (query_keys.size() != resampled_ref_points.size()) {
    // compensate last key
    constexpr double epsilon = 1e-6;
    query_keys.push_back(base_keys.back() - epsilon);
  }

  const auto query_values = autoware::interpolation::lerp(base_keys, base_values, query_keys);

  // create output reference points by updating curvature with resampled one
  std::vector<ReferencePoint> output_ref_points;
  for (size_t i = 0; i < query_values.size(); ++i) {
    output_ref_points.push_back(resampled_ref_points.at(i));
    output_ref_points.at(i).curvature = query_values.at(i);
  }

  return output_ref_points;
}

std::vector<TrajectoryPoint> resampleTrajectoryPointsPreserveYaw(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  constexpr bool enable_resampling_stop_point = true;

  if (traj_points.size() < 2) {
    return traj_points;
  }

  // First perform the standard Autoware resampling.
  // This preserves the existing behavior for position, velocity,
  // steering angles, stop points, etc.
  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(
    traj, interval, false, true, true, enable_resampling_stop_point);

  auto resampled_traj_points =
    autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
  
  if (resampled_traj_points.empty()) {
    return resampled_traj_points;
  }

  if (resampled_traj_points.size() < 2) {
    return resampled_traj_points;
  }

  // Calculate the cumulative arc length of the original trajectory.
  std::vector<double> base_arclength(traj_points.size(), 0.0);

  for (size_t i = 1; i < traj_points.size(); ++i) {
    const double dx =
      traj_points.at(i).pose.position.x -
      traj_points.at(i - 1).pose.position.x;

    const double dy =
      traj_points.at(i).pose.position.y -
      traj_points.at(i - 1).pose.position.y;

    base_arclength.at(i) =
      base_arclength.at(i - 1) + std::hypot(dx, dy);
  }

  // Extract and unwrap the original yaw.
  //
  // Unwrapping is necessary to avoid interpolation problems when
  // the yaw crosses the -pi / +pi boundary.
  std::vector<double> base_yaw(traj_points.size());

  base_yaw.at(0) = tf2::getYaw(traj_points.at(0).pose.orientation);

  for (size_t i = 1; i < traj_points.size(); ++i) {
    const double current_yaw =
      tf2::getYaw(traj_points.at(i).pose.orientation);

    const double delta_yaw =
      std::atan2(
        std::sin(current_yaw - base_yaw.at(i - 1)),
        std::cos(current_yaw - base_yaw.at(i - 1)));

    base_yaw.at(i) = base_yaw.at(i - 1) + delta_yaw;
  }

  // Calculate the cumulative arc length of the resampled trajectory.
  std::vector<double> query_arclength(resampled_traj_points.size(), 0.0);

  for (size_t i = 1; i < resampled_traj_points.size(); ++i) {
    const double dx =
      resampled_traj_points.at(i).pose.position.x -
      resampled_traj_points.at(i - 1).pose.position.x;

    const double dy =
      resampled_traj_points.at(i).pose.position.y -
      resampled_traj_points.at(i - 1).pose.position.y;

    query_arclength.at(i) =
      query_arclength.at(i - 1) + std::hypot(dx, dy);
  }

  // Interpolate the original yaw at the new arc-length positions.
  for (auto & s : query_arclength) {
    s = std::clamp(s, 0.0, base_arclength.back());
  }

  const auto interpolated_yaw =
    autoware::interpolation::lerp(
      base_arclength,
      base_yaw,
      query_arclength);

  // Replace the yaw generated by insertOrientation() with the
  // interpolated yaw from the original trajectory.
  for (size_t i = 0; i < resampled_traj_points.size(); ++i) {
    resampled_traj_points.at(i).pose.orientation =
      autoware_utils::create_quaternion_from_yaw(
        interpolated_yaw.at(i));
  }

  return resampled_traj_points;
}

void insertStopPoint(
  std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & input_stop_pose,
  const size_t stop_seg_idx)
{
  const double offset_to_segment = autoware::motion_utils::calcLongitudinalOffsetToSegment(
    traj_points, stop_seg_idx, input_stop_pose.position);

  const auto traj_spline = autoware::interpolation::SplineInterpolationPoints2d(traj_points);
  const auto stop_pose = traj_spline.getSplineInterpolatedPose(stop_seg_idx, offset_to_segment);

  if (geometry_utils::isSamePoint(traj_points.at(stop_seg_idx), stop_pose)) {
    traj_points.at(stop_seg_idx).longitudinal_velocity_mps = 0.0;
    return;
  }
  if (geometry_utils::isSamePoint(traj_points.at(stop_seg_idx + 1), stop_pose)) {
    traj_points.at(stop_seg_idx + 1).longitudinal_velocity_mps = 0.0;
    return;
  }

  TrajectoryPoint additional_traj_point;
  additional_traj_point.pose = stop_pose;
  additional_traj_point.longitudinal_velocity_mps = 0.0;

  traj_points.insert(traj_points.begin() + stop_seg_idx + 1, additional_traj_point);
}
}  // namespace trajectory_utils
}  // namespace autoware::path_optimizer
