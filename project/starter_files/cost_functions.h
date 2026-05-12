/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file cost_functions.h
 **/

#pragma once

#include <math.h>

#include <algorithm>  // std::max, min
#include <cfloat>
#include <iostream>
#include <limits>
#include <numeric>  // std::accumulate
#include <vector>

#include <carla/client/Client.h>
#include <glog/logging.h>

#include "planning_params.h"
#include "structs.h"
#include "utils.h"

using namespace std;
using namespace utils;
namespace cc = carla::client;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

// Priority levels for costs — higher cost means higher priority to avoid
const double TIME_DIFF = 1e1;
const double X_DIFF = 1e12;
const double Y_DIFF = 1e12;
const double EFFICIENCY = 1e3;
const double MAX_JERK = 1e8;
const double TOTAL_JERK = 1e7;
const double COLLISION = std::numeric_limits<double>::infinity();
const double DANGER = 1e3;
const double MAX_ACCEL = 1e8;
const double TOTAL_ACCEL = 1e8;
const double RIGHT_LANE_CHANGE = 1e1;

// Average Vehicle {Length, Width} in meters
const vector<double> VEHICLE_SIZE = {5, 2};

// Minimum following distance at zero speed
const double MIN_FOLLOW_DISTANCE = 1 * VEHICLE_SIZE[0];

// Improvement 2: Configurable cost function weights for easy tuning
struct CostWeights {
  double collision_weight = 1000.0;   // Hard penalty — avoid collision at all cost
  double proximity_weight = 0.7;      // Fraction of cost for closeness to obstacles
  double lane_center_weight = 0.3;    // Fraction of cost for deviation from lane center
};

namespace cost_functions {

double diff_cost(vector<double> coeff, double duration,
                 std::array<double, 3> goals, std::array<float, 3> sigma,
                 double cost_weight);

double collision_circles_cost_spiral(const std::vector<PathPoint>& spiral,
                                     const std::vector<State>& obstacles);

double close_to_main_goal_cost_spiral(const std::vector<PathPoint>& spiral,
                                      State goal);

}  // namespace cost_functions
