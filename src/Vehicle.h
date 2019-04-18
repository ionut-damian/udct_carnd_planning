#pragma once
#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;


class Vehicle
{
public:
    // Constructors
    Vehicle();
    Vehicle(int lane, float x, float y, float vx, float vy, float ax, float ay);

    // Destructor
    virtual ~Vehicle();

    // Vehicle functions
    void update(float x, float y, float vx, float vy, float s, float d, float ax, float ay, double theta);
    void update(float x, float y, float vx, float vy, float s, float d, float ax, float ay);
    void update(float x, float y, float vx, float vy, double theta);
    void update(float x, float y, float vx, float vy);

    Vehicle predict(double t) const;

    bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane,
        Vehicle &rVehicle) const;

    bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane,
        Vehicle &rVehicle) const;

    vector<Vehicle> generate_predictions(double horizon) const;

    void realize_next_state(Vehicle &trajectory);

    double get2DVelocity() const;
    double get2DAcceleration() const;

    // public Vehicle variables
    struct collider
    {
        bool collision; // is there a collision?
        int  time; // time collision happens
    };

    int L = 1;

    int preferred_buffer = 6; // impacts "keep lane" behavior.

    int lane;

    float x, y, vx, vy, s, d, ax, ay, theta;
};

#endif  // VEHICLE_H