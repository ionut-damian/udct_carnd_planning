#include "Vehicle.h"
#include "Cost.h"
#include "Trajectory.h"
#include "helpers.h"

#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle() {}

Vehicle::Vehicle(int lane, float x, float y, float vx, float vy, float ax, float ay)
{
    this->lane = lane;

    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->ax = ax;
    this->ay = ay;
}

Vehicle::~Vehicle() {}

void Vehicle::update(float x, float y, float vx, float vy, float s, float d)
{
    this->ax = this->vx - vx;
    this->ay = this->vy - vy;

    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;
}

void Vehicle::update(float x, float y, float vx, float vy)
{
    this->ax = this->vx - vx;
    this->ay = this->vy - vy;

    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;

    vector<double> new_frenet = getFrenet(x, y, 0, Map::getInstance()->points_x, Map::getInstance()->points_y);

    this->s = new_frenet[0];
    this->d = new_frenet[0];
}

Vehicle Vehicle::predict(int t)
{
    float new_x = this->x + this->vx*t + this->ax*t*t / 2.0f;
    float new_y = this->y + this->vy*t + this->ay*t*t / 2.0f;

    float new_vx = this->vx + this->ax*t;
    float new_vy = this->vy + this->ay*t;
    
    Vehicle new_vehicle(*this);
    new_vehicle.update(new_x, new_y, new_vx, new_vy);

    return new_vehicle;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions,
    int lane, Vehicle &rVehicle)
{
    // Returns a true if a vehicle is found behind the current vehicle, false 
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
        it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == lane && temp_vehicle.s < this->s
            && temp_vehicle.s > max_s)
        {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }

    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> &predictions,
    int lane, Vehicle &rVehicle)
{
    // Returns a true if a vehicle is found ahead of the current vehicle, false 
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int min_s = 999999;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
        it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == lane && temp_vehicle.s > this->s
            && temp_vehicle.s < min_s)
        {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }

    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon)
{
    // Generates predictions for non-ego vehicles to be used in trajectory 
    //   generation for the ego vehicle.
    vector<Vehicle> predictions;
    for (int i = 0; i < horizon; ++i)
    {
        predictions.push_back(predict(i));
    }

    return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> &trajectory)
{
    // Sets state and kinematics for ego vehicle using the last state of the trajectory.
    this->lane = trajectory[1].lane;
    update(trajectory[1].x, trajectory[1].y, trajectory[1].vx, trajectory[1].vy, trajectory[1].s, trajectory[1].d);
}

double Vehicle::get2DVelocity()
{
    return sqrt(pow(vx, 2) + pow(vy, 2));
}