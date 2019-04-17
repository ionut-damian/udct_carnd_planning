#include "Environment.h"



Environment::Environment()
{
}


Environment::~Environment()
{
}



void Environment::updateVehicle(int id, float x, float y, float vx, float vy, float s, float d)
{
    if (vehicles.find(id) == vehicles.end())
    {
        vehicles[id] = Vehicle();
    }


}