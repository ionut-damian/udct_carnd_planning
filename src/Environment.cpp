#include "Environment.h"



Environment::Environment()
{
}


Environment::~Environment()
{
}

map<int, vector<Vehicle> > Environment::generate_predictions()
{
    map<int, vector<Vehicle> > predictions;
    map<int, Vehicle>::iterator it = this->vehicles.begin();

    while (it != this->vehicles.end())
    {
        int v_id = it->first;
        vector<Vehicle> preds = it->second.generate_predictions();
        predictions[v_id] = preds;
        ++it;
    }

    return predictions;
}

void Environment::updateVehicle(int id, float x, float y, float vx, float vy, float s, float d)
{
    if (vehicles.find(id) == vehicles.end())
    {
        vehicles[id] = Vehicle();
    }

    vehicles[id].update(x, y, vx, vy, s, d, 0, 0);
}