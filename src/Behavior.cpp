#include "Behavior.h"
#include "Cost.h"


Behavior::Behavior()
{
    state = "KL";
    change_lane_left_target = -1;
    change_lane_right_target = -1;
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

    printf("> next state (");
    for (int i = 0; i < possible_states.size(); i++)
    {
        possible_traj.push_back(generate_trajectory(possible_states[i], predictions));
        if (possible_traj.back()->waypoints_x.size() == 0)
            continue;

        int intended_lane = possible_traj.back()->target_lane;
        int final_lane = (possible_states[i].compare("LCL") == 0 || possible_states[i].compare("LCR") == 0) ? intended_lane : ego.lane;

        double cost = calculate_cost(ego, predictions, possible_traj.back(), intended_lane, final_lane, state, possible_states[i]);
        printf(" | ");

        if (cost < min_cost)
        {
            min_cost = cost;
            min_cost_i = i;
        }
    }
    printf(") -> %s\n", possible_states[min_cost_i].c_str());
    
    //transition
    state = possible_states[min_cost_i];
    ego.vx = possible_traj[min_cost_i]->velocity * cos(ego.theta);
    ego.vy = possible_traj[min_cost_i]->velocity * sin(ego.theta);

    return possible_traj[min_cost_i];
}

vector<string> Behavior::successor_states()
{
    vector<string> states;
    int lane = ego.lane;

    if (state.compare("KL") == 0)
    {
        change_lane_left_target = lane -1;
        change_lane_right_target = lane +1;

        states.push_back("KL");
        if (lane != 0)
        {
            states.push_back("PLCL");
        }

        if (lane != NUM_LANES - 1)
        {
            states.push_back("PLCR");
        }
    }
    else if (state.compare("PLCL") == 0)
    {
        states.push_back("KL");
        if (lane != 0)
        {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    else if (state.compare("PLCR") == 0)
    {
        states.push_back("KL");
        if (lane != NUM_LANES - 1)
        {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    else if (state.compare("LCL") == 0)
    {
        states.push_back("KL");
        if (lane != 0 && lane != change_lane_left_target)
        {
            states.push_back("LCL");
        }
    }
    else if (state.compare("LCR") == 0)
    {
        states.push_back("KL");
        if (lane != NUM_LANES - 1 && lane != change_lane_right_target)
        {
            states.push_back("LCR");
        }
    }

    return states;
}

Trajectory* Behavior::generate_trajectory(string state, map<int, vector<Vehicle>> &predictions)
{
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    if (state.compare("KL") == 0)
    {
        return new KeepLaneTrajectory(ego, predictions);
    }
    else if (state.compare("LCL") == 0)
    {
        return new LaneChangeTrajectory(ego, change_lane_left_target, predictions);
    }
    else if (state.compare("LCR") == 0)
    {
        return new LaneChangeTrajectory(ego, change_lane_right_target, predictions);
    }
    else if (state.compare("PLCL") == 0)
    {
        return new PrepLaneChangeTrajectory(ego, change_lane_left_target, predictions);
    }
    else if (state.compare("PLCR") == 0)
    {
        return new PrepLaneChangeTrajectory(ego, change_lane_right_target, predictions);
    }
}