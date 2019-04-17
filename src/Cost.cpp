#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include "Cost.h"

using std::string;
using std::vector;

/**
 * TODO: change weights for cost functions.
 */
const float REACH_GOAL = 0.8;
const float EFFICIENCY = 0.2;

// Here we have provided two possible suggestions for cost functions, but feel 
//   free to use your own! The weighted cost over all cost functions is computed
//   in calculate_cost. The data from get_helper_data will be very useful in 
//   your implementation of the cost functions below. Please see get_helper_data
//   for details on how the helper data is computed.

float goal_distance_cost(const Trajectory* trajectory,
    const map<int, vector<Vehicle>> &predictions,
    map<string, float> &data)
{
    // Cost increases based on distance of intended lane (for planning a lane 
    //   change) and final lane of trajectory.
    // Cost of being out of goal lane also becomes larger as vehicle approaches 
    //   goal distance.
    // This function is very similar to what you have already implemented in the 
    //   "Implement a Cost Function in C++" quiz.
    float cost;
    float distance = data["distance_to_goal"];
    if (distance > 0)
    {
        cost = 1 - 2 * exp(-(abs(2.0*data["goal_lane"] - data["intended_lane"]
            - data["final_lane"]) / distance));
    }
    else
    {
        cost = 1;
    }

    return cost;
}

float inefficiency_cost(const Trajectory* trajectory,
    const map<int, vector<Vehicle>> &predictions,
    map<string, float> &data)
{
    // Cost becomes higher for trajectories with intended lane and final lane 
    //   that have traffic slower than vehicle's target speed.
    // You can use the lane_speed function to determine the speed for a lane. 
    // This function is very similar to what you have already implemented in 
    //   the "Implement a Second Cost Function in C++" quiz.

    if (data["intended_lane"] < 0 || data["intended_lane"] > 3)
        return 1;

    if (data["final_lane"] < 0 || data["final_lane"] > 3)
        return 1;

    float target_speed = data["tarrget_speed"];

    float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0)
    {
        proposed_speed_intended = target_speed;
    }

    float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0)
    {
        proposed_speed_final = target_speed;
    }

    float cost = (2.0*target_speed - proposed_speed_intended
        - proposed_speed_final) / target_speed;

    return cost;
}

float lane_speed(const map<int, vector<Vehicle>> &predictions, int lane)
{
    // All non ego vehicles in a lane have the same speed, so to get the speed 
    //   limit for a lane, we can just find one vehicle in that lane.
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin();
        it != predictions.end(); ++it)
    {
        int key = it->first;
        Vehicle vehicle = it->second[0];
        if (vehicle.lane == lane && key != -1)
        {
            return vehicle.get2DVelocity();
        }
    }
    // Found no vehicle in the lane
    return -1.0;
}

float calculate_cost(const map<int, vector<Vehicle>> &predictions,
    const Trajectory* trajectory, int goal_lane, float goal_s, float target_speed)
{
    // Sum weighted cost functions to get total cost for trajectory.
    map<string, float> trajectory_data = get_helper_data(trajectory,
        predictions, goal_lane, goal_s, target_speed);
    float cost = 0.0;

    // Add additional cost functions here.
    vector<std::function<float(const Trajectory*,
        const map<int, vector<Vehicle>> &,
        map<string, float> &)
        >> cf_list = { goal_distance_cost, inefficiency_cost };
    vector<float> weight_list = { REACH_GOAL, EFFICIENCY };

    for (int i = 0; i < cf_list.size(); ++i)
    {
        float new_cost = weight_list[i] * cf_list[i](trajectory, predictions,
            trajectory_data);
        cost += new_cost;
    }

    return cost;
}

map<string, float> get_helper_data(const Trajectory* trajectory,
    const map<int, vector<Vehicle>> &predictions, int goal_lane, int goal_s, float target_speed)
{
    // Generate helper data to use in cost functions:
    // intended_lane: the current lane +/- 1 if vehicle is planning or 
    //   executing a lane change.
    // final_lane: the lane of the vehicle at the end of the trajectory.
    // distance_to_goal: the distance of the vehicle to the goal.

    // Note that intended_lane and final_lane are both included to help 
    //   differentiate between planning and executing a lane change in the 
    //   cost functions.
    map<string, float> trajectory_data;
    Vehicle trajectory_last = trajectory->waypoints.back();
    float intended_lane = trajectory->target_lane;

    float distance_to_goal = goal_s - trajectory_last.s;
    float final_lane = trajectory_last.lane;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;

    trajectory_data["goal_lane"] = goal_lane;
    trajectory_data["goal_s"] = goal_s;
    trajectory_data["target_speed"] = target_speed;

    return trajectory_data;
}