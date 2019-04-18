#pragma once
#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "Vehicle.h"
#include "Trajectory.h"
#include "Map.h"
#include "Environment.h"

class Behavior
{
public:
    Behavior();
    ~Behavior();

    Trajectory* generate_trajectory(string state, map<int, vector<Vehicle>> &predictions);

    vector<string> successor_states();

    Trajectory* choose_next_state(map<int, vector<Vehicle>> &predictions);

    string state;
    int change_lane_left_target;
    int change_lane_right_target;

    Vehicle ego;
};

#endif

