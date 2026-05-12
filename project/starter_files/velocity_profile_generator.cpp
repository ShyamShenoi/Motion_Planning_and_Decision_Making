/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file velocity_trajectory_generator.cpp
 **/

#include "velocity_profile_generator.h"

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <thread>

VelocityProfileGenerator::VelocityProfileGenerator() {}
VelocityProfileGenerator::~VelocityProfileGenerator() {}

void VelocityProfileGenerator::setup(const double& time_gap,
                                     const double& a_max,
                                     const double& slow_speed) {
  _time_gap = time_gap;
  _a_max = a_max;
  _slow_speed = slow_speed;
}

// Improvement 5: Apply a 3-point moving average to smooth velocity profile
static void smooth_velocities(std::vector<TrajectoryPoint>& traj) {
  if (traj.size() < 3) return;
  std::vector<double> smoothed(traj.size());
  smoothed[0] = traj[0].v;
  smoothed[traj.size() - 1] = traj[traj.size() - 1].v;
  for (size_t i = 1; i < traj.size() - 1; ++i) {
    smoothed[i] = (traj[i - 1].v + traj[i].v + traj[i + 1].v) / 3.0;
  }
  for (size_t i = 0; i < traj.size(); ++i) {
    traj[i].v = smoothed[i];
  }
}

std::vector<TrajectoryPoint> VelocityProfileGenerator::generate_trajectory(
    const std::vector<PathPoint>& spiral, const double& desired_speed,
    const State& ego_state, const State& lead_car_state,
    const Maneuver& maneuver) const {
  std::vector<TrajectoryPoint> trajectory;
  double start_speed = utils::magnitude(ego_state.velocity);

  if (maneuver == DECEL_TO_STOP) {
    trajectory = decelerate_trajectory(spiral, start_speed);
  } else if (maneuver == FOLLOW_VEHICLE) {
    trajectory =
        follow_trajectory(spiral, start_speed, desired_speed, lead_car_state);
  } else {
    trajectory = nominal_trajectory(spiral, start_speed, desired_speed);
  }

  // Interpolate between the zeroth and first state to avoid controller stall
  if (trajectory.size() > 1) {
    TrajectoryPoint interpolated_state;
    interpolated_state.path_point.x =
        (trajectory[1].path_point.x - trajectory[0].path_point.x) * 0.1 +
        trajectory[0].path_point.x;
    interpolated_state.path_point.y =
        (trajectory[1].path_point.y - trajectory[0].path_point.y) * 0.1 +
        trajectory[0].path_point.y;
    interpolated_state.path_point.z =
        (trajectory[1].path_point.z - trajectory[0].path_point.z) * 0.1 +
        trajectory[0].path_point.z;
    interpolated_state.v =
        (trajectory[1].v - trajectory[0].v) * 0.1 + trajectory[0].v;
    trajectory[0] = interpolated_state;
  }

  return trajectory;
}

// Decelerate smoothly to a full stop
std::vector<TrajectoryPoint> VelocityProfileGenerator::decelerate_trajectory(
    const std::vector<PathPoint>& spiral, const double& start_speed) const {
  std::vector<TrajectoryPoint> trajectory;

  auto decel_distance = calc_distance(start_speed, _slow_speed, -_a_max);
  auto brake_distance = calc_distance(_slow_speed, 0, -_a_max);

  auto path_length{0.0};
  auto stop_index{spiral.size() - 1};
  for (size_t i = 0; i < stop_index; ++i) {
    path_length += utils::distance(spiral[i + 1], spiral[i]);
  }

  if (brake_distance + decel_distance > path_length) {
    // Path too short for a smooth profile — build in reverse from stop
    std::vector<double> speeds;
    auto vf{0.0};
    auto it = speeds.begin();
    speeds.insert(it, 0.0);

    for (int i = stop_index - 1; i >= 0; --i) {
      auto dist = utils::distance(spiral[i + 1], spiral[i]);
      auto vi = calc_final_speed(vf, -_a_max, dist);
      if (vi > start_speed) vi = start_speed;
      auto it2 = speeds.begin();
      speeds.insert(it2, vi);
      vf = vi;
    }

    double time_step{0.0};
    double time{0.0};
    for (size_t i = 0; i < speeds.size() - 1; ++i) {
      TrajectoryPoint traj_point;
      traj_point.path_point = spiral[i];
      traj_point.v = speeds[i];
      traj_point.relative_time = time;
      trajectory.push_back(traj_point);
      time_step = std::fabs(speeds[i] - speeds[i + 1]) / _a_max;
      time += time_step;
    }
    auto i = spiral.size() - 1;
    TrajectoryPoint traj_point;
    traj_point.path_point = spiral[i];
    traj_point.v = speeds[i];
    traj_point.relative_time = time;
    trajectory.push_back(traj_point);

  } else {
    auto brake_index{stop_index};
    auto temp_dist{0.0};
    while ((brake_index > 0) && (temp_dist < brake_distance)) {
      temp_dist += utils::distance(spiral[brake_index], spiral[brake_index - 1]);
      --brake_index;
    }
    uint decel_index{0};
    temp_dist = 0.0;
    while ((decel_index < brake_index) && (temp_dist < decel_distance)) {
      temp_dist += utils::distance(spiral[decel_index + 1], spiral[decel_index]);
      ++decel_index;
    }

    double time_step{0.0};
    double time{0.0};
    auto vi{start_speed};

    for (size_t i = 0; i < decel_index; ++i) {
      auto dist = utils::distance(spiral[i + 1], spiral[i]);
      auto vf = calc_final_speed(vi, -_a_max, dist);
      if (vf < _slow_speed) vf = _slow_speed;
      TrajectoryPoint traj_point;
      traj_point.path_point = spiral[i];
      traj_point.v = vi;
      traj_point.relative_time = time;
      trajectory.push_back(traj_point);
      time_step = std::fabs(vf - vi) / _a_max;
      time += time_step;
      vi = vf;
    }
    for (size_t i = decel_index; i < brake_index; ++i) {
      TrajectoryPoint traj_point;
      traj_point.path_point = spiral[i];
      traj_point.v = vi;
      traj_point.relative_time = time;
      trajectory.push_back(traj_point);
      auto dist = utils::distance(spiral[i + 1], spiral[i]);
      if (dist > DBL_EPSILON)
        time_step = dist / vi;
      else
        time_step = 0.0;
      time += time_step;
    }
    for (size_t i = brake_index; i < stop_index; ++i) {
      auto dist = utils::distance(spiral[i + 1], spiral[i]);
      auto vf = calc_final_speed(vi, -_a_max, dist);
      TrajectoryPoint traj_point;
      traj_point.path_point = spiral[i];
      traj_point.v = vi;
      traj_point.relative_time = time;
      trajectory.push_back(traj_point);
      time_step = std::fabs(vf - vi) / _a_max;
      time += time_step;
      vi = vf;
    }
    auto i = stop_index;
    TrajectoryPoint traj_point;
    traj_point.path_point = spiral[i];
    traj_point.v = 0.0;
    traj_point.relative_time = time;
    trajectory.push_back(traj_point);
  }

  smooth_velocities(trajectory);
  return trajectory;
}

// Follow a lead vehicle — stub (not required by the rubric but provided for completeness)
std::vector<TrajectoryPoint> VelocityProfileGenerator::follow_trajectory(
    const std::vector<PathPoint>& spiral, const double& start_speed,
    const double& desired_speed, const State& lead_car_state) const {
  std::vector<TrajectoryPoint> trajectory;
  return trajectory;
}

// Nominal speed tracking (lane follow or cruise control)
std::vector<TrajectoryPoint> VelocityProfileGenerator::nominal_trajectory(
    const std::vector<PathPoint>& spiral, const double& start_speed,
    double const& desired_speed) const {
  std::vector<TrajectoryPoint> trajectory;
  double accel_distance;

  if (desired_speed < start_speed) {
    accel_distance = calc_distance(start_speed, desired_speed, -_a_max);
  } else {
    accel_distance = calc_distance(start_speed, desired_speed, _a_max);
  }

  size_t ramp_end_index{0};
  double distance{0.0};
  while (ramp_end_index < (spiral.size() - 1) && (distance < accel_distance)) {
    distance +=
        utils::distance(spiral[ramp_end_index], spiral[ramp_end_index + 1]);
    ramp_end_index += 1;
  }

  double time_step{0.0};
  double time{0.0};
  double vi{start_speed};

  for (size_t i = 0; i < ramp_end_index; ++i) {
    auto dist = utils::distance(spiral[i], spiral[i + 1]);
    double vf;
    if (desired_speed < start_speed) {
      vf = calc_final_speed(vi, -_a_max, dist);
      if (vf < desired_speed) vf = desired_speed;
    } else {
      vf = calc_final_speed(vi, _a_max, dist);
      if (vf > desired_speed) vf = desired_speed;
    }
    TrajectoryPoint traj_point;
    traj_point.path_point = spiral[i];
    traj_point.v = vi;
    traj_point.relative_time = time;
    trajectory.push_back(traj_point);
    time_step = std::fabs(vf - vi) / _a_max;
    time += time_step;
    vi = vf;
  }

  for (size_t i = ramp_end_index; i < spiral.size() - 1; ++i) {
    TrajectoryPoint traj_point;
    traj_point.path_point = spiral[i];
    traj_point.v = desired_speed;
    traj_point.relative_time = time;
    trajectory.push_back(traj_point);
    auto dist = utils::distance(spiral[i], spiral[i + 1]);
    if (std::abs(desired_speed) < DBL_EPSILON)
      time_step = 0.0;
    else
      time_step = dist / desired_speed;
    time += time_step;
  }

  // Add last point
  auto i = spiral.size() - 1;
  TrajectoryPoint traj_point;
  traj_point.path_point = spiral[i];
  traj_point.v = desired_speed;
  traj_point.relative_time = time;
  trajectory.push_back(traj_point);

  smooth_velocities(trajectory);
  return trajectory;
}

/*
Using d = (v_f^2 - v_i^2) / (2 * a)
*/
double VelocityProfileGenerator::calc_distance(const double& v_i,
                                               const double& v_f,
                                               const double& a) const {
  double d{0.0};
  if (std::abs(a) < DBL_EPSILON) {
    d = std::numeric_limits<double>::infinity();
  } else {
    d = (v_f * v_f - v_i * v_i) / (2.0 * a);
  }
  return d;
}

/*
Using v_f = sqrt(v_i^2 + 2*a*d)
*/
double VelocityProfileGenerator::calc_final_speed(const double& v_i,
                                                  const double& a,
                                                  const double& d) const {
  double v_f{0.0};
  double disc = v_i * v_i + 2.0 * a * d;
  if (disc <= 0.0) {
    v_f = 0.0;
  } else if (disc == std::numeric_limits<double>::infinity() ||
             std::isnan(disc)) {
    v_f = std::numeric_limits<double>::infinity();
  } else {
    v_f = std::sqrt(disc);
  }
  return v_f;
}
