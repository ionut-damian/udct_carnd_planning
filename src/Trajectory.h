#pragma once
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "Vehicle.h"
#include "Map.h"

class Trajectory
{
public:
    Trajectory();
    ~Trajectory();

    Vehicle get_kinematics(Vehicle& vehicle, int lane, double t, map<int, vector<Vehicle>> &predictions, float max_acceleration, float target_speed);
    double computeVelocty(Vehicle& vehicle, int lane, double t, map<int, vector<Vehicle>> &predictions, float max_acceleration, float target_speed);

    vector<Vehicle> waypoints;

    vector<double> waypoints_x;
    vector<double> waypoints_y;

    int target_lane;
};

class ConstantAccTrajectory : public Trajectory
{
public:
    ConstantAccTrajectory(Vehicle& vehicle);
};

class KeepLaneTrajectory : public Trajectory
{
public:
    KeepLaneTrajectory(Vehicle& vehicle, map<int, vector<Vehicle>> &predictions, float max_acceleration, float target_speed);
};

class PrepLaneChangeTrajectory : public Trajectory
{
public:
    PrepLaneChangeTrajectory(Vehicle& vehicle, int target_lane, map<int, vector<Vehicle>> &predictions, float max_acceleration, float target_speed);
};

class LaneChangeTrajectory : public Trajectory
{
public:
    LaneChangeTrajectory(Vehicle& vehicle, int target_lane, map<int, vector<Vehicle>> &predictions, float max_acceleration, float target_speed);
};

#endif