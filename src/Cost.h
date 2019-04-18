#pragma once
#ifndef COST_H
#define COST_H

#include "Vehicle.h"
#include "Trajectory.h"

using std::map;
using std::string;
using std::vector;

float calculate_cost(const Vehicle& ego, const map<int, vector<Vehicle>> &predictions,
    const Trajectory* trajectory, int intended_lane, int final_lane, string old_state, string new_state);

float collision_cost(const Vehicle& ego, const Trajectory* trajectory,
    const map<int, vector<Vehicle>> &predictions, int intended_lane, int final_lane, string old_state, string new_state);

float lane_change_cost(const Vehicle& ego, const Trajectory* trajectory,
    const map<int, vector<Vehicle>> &predictions, int intended_lane, int final_lane, string old_state, string new_state);

float inefficiency_cost(const Vehicle& ego, const Trajectory* trajectory,
    const map<int, vector<Vehicle>> &predictions, int intended_lane, int final_lane, string old_state, string new_state);

float lazyness_cost(const Vehicle& ego, const Trajectory* trajectory,
    const map<int, vector<Vehicle>> &predictions, int intended_lane, int final_lane, string old_state, string new_state);

bool nearest_vehicle(double ego_s, const map<int, vector<Vehicle>> &predictions, int lane, Vehicle& vehicle, float horizon);
float lane_speed(double ego_s, const map<int, vector<Vehicle>> &predictions, int lane);

#endif  // COST_H