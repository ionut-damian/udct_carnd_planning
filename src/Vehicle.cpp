#include "Vehicle.h"
#include "Cost.h"
#include "Trajectory.h"
#include "helpers.h"
#include "config.h"

#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle() 
{
    this->lane = 0;
    this->x = 0;
    this->y = 0;
    this->vx = 0;
    this->vy = 0;
    this->ax = 0;
    this->ay = 0;
    this->theta = 0;
    this->s = 0;
    this->d = 0;
}

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

void Vehicle::update(float x, float y, float vx, float vy, float s, float d, float ax, float ay, double theta)
{
    this->theta = theta;

    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;
    this->ax = ax;
    this->ay = ay;

    this->lane = floor(this->d / LANE_WIDTH);

    //snap lane to min or max to force car to go back
    if (this->lane < 0)
        this->lane = 0;
    if (this->lane > NUM_LANES - 1)
        this->lane = NUM_LANES - 1;
}

void Vehicle::update(float x, float y, float vx, float vy, float s, float d, float ax, float ay)
{
    this->theta = atan2(y - this->y, x - this->x);
    if (theta < 0)
        theta = 360 - abs(theta);

    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;
    this->ax = ax;
    this->ay = ay;

    this->lane = (int)this->d / (int)LANE_WIDTH;

    //snap lane to min or max to force car to go back
    if (this->lane < 0)
       this->lane = 0;
    if (this->lane > NUM_LANES - 1)
        this->lane = NUM_LANES - 1;
}

void Vehicle::update(float x, float y, float vx, float vy, double theta)
{
    vector<double> new_frenet = getFrenet(x, y, theta, Map::getInstance()->points_x, Map::getInstance()->points_y);
    update(x, y, vx, vy, new_frenet[0], new_frenet[1], 0, 0, theta);
}

void Vehicle::update(float x, float y, float vx, float vy)
{
    this->theta = atan2(y - this->y, x - this->x);
    if (theta < 0)
        theta = 360 - abs(theta);

    vector<double> new_frenet = getFrenet(x, y, this->theta, Map::getInstance()->points_x, Map::getInstance()->points_y);

    update(x, y, vx, vy, new_frenet[0], new_frenet[1], 0, 0);
}

Vehicle Vehicle::predict(double t) const
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
    int lane, Vehicle &rVehicle) const
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
    int lane, Vehicle &rVehicle) const
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

vector<Vehicle> Vehicle::generate_predictions(double horizon) const
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

void Vehicle::realize_next_state(Vehicle &trajectory)
{
    // Sets state and kinematics for ego vehicle using the last state of the trajectory.
    this->lane = trajectory.lane;
    update(trajectory.x, trajectory.y, trajectory.vx, trajectory.vy, trajectory.s, trajectory.d, trajectory.ax, trajectory.ay);
}

double Vehicle::get2DVelocity() const
{
    return sqrt(pow(vx, 2) + pow(vy, 2));
}

double Vehicle::get2DAcceleration() const
{
    return sqrt(pow(ax, 2) + pow(ay, 2));
}