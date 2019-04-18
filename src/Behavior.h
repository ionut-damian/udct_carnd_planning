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
    Behavior(int numLanes, float target_speed, float max_acceleration, int goal_lane, float goal_s);
    ~Behavior();

    Trajectory* generate_trajectory(string state, map<int, vector<Vehicle>> &predictions);

    vector<string> successor_states();

    Trajectory* choose_next_state(Environment& environment);

    float target_speed, max_acceleration, goal_s;
    int lanes_available, goal_lane;

    string state;
    Vehicle ego;
};

#endif

