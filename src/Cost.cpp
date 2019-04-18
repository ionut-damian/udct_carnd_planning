#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include "Cost.h"
#include "helpers.h"
#include "config.h"

using std::string;
using std::vector;

const float COLLISION = 0.6;
const float LANE_CHANGE = 0.6;
const float EFFICIENCY = 0.4;
const float DISTANCE = 0.4;
const float LAZY = 0.07;

float lazyness_cost(const Vehicle& ego, const Trajectory* trajectory, const map<int, vector<Vehicle>> &predictions, int intended_lane, int final_lane, string old_state, string new_state)
{
    if (old_state.compare(new_state) == 0)
        return 0;
    else if ((old_state.compare("PLCL") == 0 || old_state.compare("PLCR") == 0) && (new_state.compare("LCL") == 0 || new_state.compare("LCR") == 0))
        return 0; //encourage to jump out of P* states
    else
        return 1;
}

float collision_cost(const Vehicle& ego, const Trajectory* trajectory, const map<int, vector<Vehicle>> &predictions, int intended_lane, int final_lane, string old_state, string new_state)
{
    for (int i = 0; i < trajectory->waypoints_x.size(), i < predictions.at(0).size(); i++)
    {
        for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it)
        {
            int key = it->first;
            Vehicle vehicle = it->second[i];
            if (distance(trajectory->waypoints_x[i], trajectory->waypoints_y[i], vehicle.x, vehicle.y) < CAR_RADIUS)
            {
                return 1;
            }
        }
    }

    return 0;
}

float lane_change_cost(const Vehicle& ego, const Trajectory* trajectory, const map<int, vector<Vehicle>> &predictions, int intended_lane, int final_lane, string old_state, string new_state)
{
    if (new_state.compare("KL") == 0)
        return 0;

    float spot_size = 30;
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        Vehicle temp_vehicle = it->second[0];
        if (temp_vehicle.lane == intended_lane && temp_vehicle.s > ego.s - spot_size / 2 && temp_vehicle.s < ego.s + spot_size / 2)
        {
            return 1;
        }
    }

    return 0;
}

float distance_to_vehicle_cost(const Vehicle& ego, const Trajectory* trajectory, const map<int, vector<Vehicle>> &predictions, int intended_lane, int final_lane, string old_state, string new_state)
{
    if (intended_lane < 0 || intended_lane >= NUM_LANES)
        return 1;
    if (final_lane < 0 || final_lane >= NUM_LANES)
        return 1;

    float horizon = 200;

    Vehicle nearest_intended;
    float dist_intended = horizon;
    if (nearest_vehicle(ego.s, predictions, intended_lane, nearest_intended, horizon))
        dist_intended = nearest_intended.s - ego.s;

    Vehicle nearest_final;
    float dist_final = horizon;
    if (nearest_vehicle(ego.s, predictions, final_lane, nearest_final, horizon))
        dist_final = nearest_final.s - ego.s;

    return 2.0 - (dist_intended + dist_final) / horizon;
}

float inefficiency_cost(const Vehicle& ego, const Trajectory* trajectory, const map<int, vector<Vehicle>> &predictions, int intended_lane, int final_lane, string old_state, string new_state)
{
    // Cost becomes higher for trajectories with intended lane and final lane 
    //   that have traffic slower than vehicle's target speed.
    // You can use the lane_speed function to determine the speed for a lane. 
    // This function is very similar to what you have already implemented in 
    //   the "Implement a Second Cost Function in C++" quiz.

    if (intended_lane < 0 || intended_lane > NUM_LANES)
        return 1;
    if (final_lane < 0 || final_lane > NUM_LANES)
        return 1;

    float target_speed = MAX_SPEED;

    float proposed_speed_intended = lane_speed(ego.s, predictions, intended_lane);
    if (proposed_speed_intended < 0)
    {
        proposed_speed_intended = target_speed;
    }

    float proposed_speed_final = lane_speed(ego.s, predictions, final_lane);
    if (proposed_speed_final < 0)
    {
        proposed_speed_final = target_speed;
    }

    float cost = (2.0*target_speed - proposed_speed_intended
        - proposed_speed_final) / target_speed;

    return cost;
}

bool nearest_vehicle(double ego_s, const map<int, vector<Vehicle>> &predictions, int lane, Vehicle& vehicle, float horizon)
{
    // All non ego vehicles in a lane have the same speed, so to get the speed 
    //   limit for a lane, we can just find one vehicle in that lane.
    Vehicle nearest;
    bool found = false;
    double min_s = 999999;
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        Vehicle vehicle = it->second[0];
        if (vehicle.lane == lane
            && vehicle.s > ego_s && vehicle.s < ego_s + horizon //the next car on the lane in front of us and within Xm
            && vehicle.s < min_s)
        {
            min_s = vehicle.s;
            nearest = vehicle;
            found = true;
        }
    }

    if (found)
    {
        vehicle = nearest;
        return true;
    }
    else
    {
        return false;
    }
}

float lane_speed(double ego_s, const map<int, vector<Vehicle>> &predictions, int lane)
{
    // All non ego vehicles in a lane have the same speed, so to get the speed 
    //   limit for a lane, we can just find one vehicle in that lane.
    Vehicle nearest;   
    if (nearest_vehicle(ego_s, predictions, lane, nearest, 200))
    {
        return nearest.get2DVelocity();
    }
    else
    {
        return -1.0;
    }
}

float calculate_cost(const Vehicle& ego, const map<int, vector<Vehicle>> &predictions, const Trajectory* trajectory, int intended_lane, int final_lane, string old_state, string new_state)
{
    // Sum weighted cost functions to get total cost for trajectory.
    float cost = 0.0;

    // Add additional cost functions here.
    vector<std::function<float(const Vehicle&, const Trajectory*, const map<int, vector<Vehicle>> &, int, int, string, string)
        >> cf_list = { lane_change_cost, distance_to_vehicle_cost, lazyness_cost };
    vector<float> weight_list = { LANE_CHANGE, DISTANCE, LAZY };

    for (int i = 0; i < cf_list.size(); ++i)
    {
        float new_cost = weight_list[i] * cf_list[i](ego, trajectory, predictions, intended_lane, final_lane, old_state, new_state);
        printf("%.2f ", new_cost);
        cost += new_cost;
    }

    return cost;
}