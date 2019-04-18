#pragma once
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "Vehicle.h"
#include "Map.h"
#include "config.h"

class Trajectory
{
public:
    Trajectory();
    ~Trajectory();

    double computeVelocty(const Vehicle& vehicle, int lane, double t, map<int, vector<Vehicle>> &predictions, float max_velocity = MAX_SPEED);
    void buildTrajectory(const Vehicle& vehicle, vector<double> anchors_x, vector<double> anchors_y, double velocity, double trajectory_length);

    vector<double> waypoints_x;
    vector<double> waypoints_y;

    int target_lane;
    double velocity;
};

class KeepLaneTrajectory : public Trajectory
{
public:
    KeepLaneTrajectory(const Vehicle& vehicle, map<int, vector<Vehicle>> &predictions);
};

class PrepLaneChangeTrajectory : public Trajectory
{
public:
    PrepLaneChangeTrajectory(const Vehicle& vehicle, int target_lane, map<int, vector<Vehicle>> &predictions);
};

class LaneChangeTrajectory : public Trajectory
{
public:
    LaneChangeTrajectory(const Vehicle& vehicle, int target_lane, map<int, vector<Vehicle>> &predictions);
};

#endif