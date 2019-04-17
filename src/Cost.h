#pragma once
#ifndef COST_H
#define COST_H

#include "Vehicle.h"
#include "Trajectory.h"

using std::map;
using std::string;
using std::vector;

float calculate_cost(const map<int, vector<Vehicle>> &predictions,
    const Trajectory* trajectory, int goal_lane, float goal_s, float target_speed);

float goal_distance_cost(const Trajectory* trajectory,
    const map<int, vector<Vehicle>> &predictions,
    map<string, float> &data);

float inefficiency_cost(const Trajectory* trajectory,
    const map<int, vector<Vehicle>> &predictions,
    map<string, float> &data);

float lane_speed(const map<int, vector<Vehicle>> &predictions, int lane);

map<string, float> get_helper_data(const Trajectory* trajectory,
    const map<int, vector<Vehicle>> &predictions, int goal_lane, int goal_s, float target_speed);

#endif  // COST_H