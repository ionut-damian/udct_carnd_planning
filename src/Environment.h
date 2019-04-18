#pragma once
#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <map>
#include "Vehicle.h"

class Environment
{
public:
    Environment();
    ~Environment();

    void updateVehicle(int id, float x, float y, float vx, float vy, float s, float d);

    map<int, vector<Vehicle> > generate_predictions();

    map<int, Vehicle> vehicles;
};

#endif