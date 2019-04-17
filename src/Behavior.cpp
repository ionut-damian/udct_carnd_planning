#include "Behavior.h"
#include "Cost.h"


Behavior::Behavior(int numLanes, float target_speed, float max_acceleration, int goal_lane, float goal_s) :
    lanes_available(numLanes), target_speed(target_speed), max_acceleration(max_acceleration), goal_s(goal_s), goal_lane(goal_lane)
{
}

Behavior::~Behavior()
{
}

Trajectory* Behavior::choose_next_state(map<int, vector<Vehicle>> &predictions)
{
    vector<string> possible_states = successor_states();
    vector<Trajectory*> possible_traj;

    float min_cost = 9999;
    int min_cost_i = 0;

    for (int i = 0; i < possible_states.size(); i++)
    {
        possible_traj.push_back(generate_trajectory(possible_states[i], predictions));
        if (possible_traj.back()->waypoints.size() == 0)
            continue;

        double cost = calculate_cost(predictions, possible_traj.back(), goal_lane, goal_s, target_speed);

        if (cost < min_cost)
        {
            min_cost = cost;
            min_cost_i = i;
        }
    }

    printf("> next state = %s\n", possible_states[min_cost_i].c_str());

    return possible_traj[min_cost_i];
}

vector<string> Behavior::successor_states()
{
    // Provides the possible next states given the current state for the FSM 
    //   discussed in the course, with the exception that lane changes happen 
    //   instantaneously, so LCL and LCR can only transition back to KL.
    vector<string> states;
    states.push_back("KL");

    int lane = ego.lane;

    if (state.compare("KL") == 0)
    {
        if (lane != lanes_available - 1)
        {
            states.push_back("PLCL");
        }

        if (lane != 0)
        {
            states.push_back("PLCR");
        }
    }
    else if (state.compare("PLCL") == 0)
    {
        if (lane != lanes_available - 1)
        {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    else if (state.compare("PLCR") == 0)
    {
        if (lane != 0)
        {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }

    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}

Trajectory* Behavior::generate_trajectory(string state, map<int, vector<Vehicle>> &predictions)
{
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    if (state.compare("CA") == 0)
    {
        return new ConstantAccTrajectory(ego);
    }
    else if (state.compare("KL") == 0)
    {
        return new KeepLaneTrajectory(ego, predictions, max_acceleration, target_speed);
    }
    else if (state.compare("LCL") == 0 == 0)
    {
        return new LaneChangeTrajectory(ego, ego.lane -1, predictions, max_acceleration, target_speed);
    }
    else if (state.compare("LCR") == 0 == 0)
    {
        return new LaneChangeTrajectory(ego, ego.lane + 1, predictions, max_acceleration, target_speed);
    }
    else if (state.compare("PLCL") == 0 == 0)
    {
        return new PrepLaneChangeTrajectory(ego, ego.lane - 1, predictions, max_acceleration, target_speed);
    }
    else if (state.compare("PLCR") == 0)
    {
        return new PrepLaneChangeTrajectory(ego, ego.lane + 1, predictions, max_acceleration, target_speed);
    }
}