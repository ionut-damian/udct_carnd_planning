#include "Trajectory.h"
#include <algorithm>

#include "helpers.h"


Trajectory::Trajectory()
{
}


Trajectory::~Trajectory()
{
}

ConstantAccTrajectory::ConstantAccTrajectory(Vehicle& vehicle)
{
    target_lane = vehicle.lane;

    // Generate a constant speed trajectory.
    waypoints = { Vehicle(vehicle),
                  vehicle.predict(1) };
}

KeepLaneTrajectory::KeepLaneTrajectory(Vehicle& vehicle, map<int, vector<Vehicle>> &predictions, float max_acceleration, float target_speed)
{
    target_lane = vehicle.lane;

    // Generate a keep lane trajectory.
    waypoints.push_back(Vehicle(vehicle)); //current position
    waypoints.push_back(get_kinematics(vehicle, predictions, max_acceleration, target_speed)); //next positio
}

PrepLaneChangeTrajectory::PrepLaneChangeTrajectory(Vehicle& vehicle, int target_lane, map<int, vector<Vehicle>> &predictions, float max_acceleration, float target_speed)
{
    this->target_lane = target_lane;

    // Generate a trajectory preparing for a lane change.
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;

    waypoints.push_back(Vehicle(vehicle)); //current position
    waypoints.push_back(get_kinematics(vehicle, predictions, max_acceleration, target_speed)); //next position


    // disable speed adjustment since it leads to deadlock wihtout a free spot detection
    //if (get_vehicle_behind(predictions, vehicle.lane, vehicle_behind))
    //{
    //    // Keep speed of current lane so as not to collide with car behind.
    //    new_s = curr_lane_new_kinematics[0];
    //    new_v = curr_lane_new_kinematics[1];
    //    new_a = curr_lane_new_kinematics[2];
    //}
    //else
    //{
    //    vector<float> best_kinematics;
    //    vector<float> next_lane_new_kinematics = get_kinematics(predictions, target_lane);
    //    // Choose kinematics with lowest velocity.
    //    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1])
    //    {
    //        best_kinematics = next_lane_new_kinematics;
    //    }
    //    else
    //    {
    //        best_kinematics = curr_lane_new_kinematics;
    //    }
    //    new_s = best_kinematics[0];
    //    new_v = best_kinematics[1];
    //    new_a = best_kinematics[2];
    //}

    //waypoints.push_back(Vehicle(vehicle.lane, new_s, new_v, new_a, vehicle.state));
}

LaneChangeTrajectory::LaneChangeTrajectory(Vehicle& vehicle, int target_lane, map<int, vector<Vehicle>> &predictions, float max_acceleration, float target_speed)
{
    this->target_lane = target_lane;

    // Generate a lane change trajectory.
    Vehicle next_lane_vehicle;

    // Check if a lane change is possible (check if another vehicle occupies 
    //   that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
        it != predictions.end(); ++it)
    {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == vehicle.s && next_lane_vehicle.lane == target_lane)
        {
            // If lane change is not possible, return empty trajectory.
            return;
        }
    }

    waypoints.push_back(Vehicle(vehicle)); //current position
    waypoints.push_back(get_kinematics(vehicle, predictions, max_acceleration, target_speed)); //next position
}

Vehicle Trajectory::get_kinematics(Vehicle& vehicle, map<int, vector<Vehicle>> &predictions, float max_acceleration, float target_speed)
{
    // Gets next timestep kinematics (position, velocity, acceleration) 
    //   for a given lane. Tries to choose the maximum velocity and acceleration, 
    //   given other vehicle positions and accel/velocity constraints.
    float max_velocity_accel_limit_x = max_acceleration + vehicle.vx;
    float max_velocity_accel_limit_y = max_acceleration + vehicle.vy;
    float new_position_x, new_position_y;
    float new_velocity_x, new_velocity_y;
    float new_accel_x, new_accel_y;
    
    float d = vehicle.lane * 4.0 + 2.0;
    
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;


    if (vehicle.get_vehicle_ahead(predictions, vehicle.lane, vehicle_ahead))
    {
        if (vehicle.get_vehicle_behind(predictions, vehicle.lane, vehicle_behind))
        {
            // must travel at the speed of traffic, regardless of preferred buffer
            new_velocity_x = vehicle_ahead.vx;
            new_velocity_y = vehicle_ahead.vy;
        }
        else
        {
            double max_pos_s = vehicle_ahead.s - vehicle.s - vehicle.preferred_buffer;
            vector<double> max_pos = getXY(max_pos_s, vehicle.lane * 4.0 + 2.0, Map::getInstance()->points_s, Map::getInstance()->points_y, Map::getInstance()->points_y);

            float max_velocity_in_front_x = max_pos[0] + vehicle_ahead.vx - 0.5 * (vehicle.ax);
            float max_velocity_in_front_y = max_pos[1] + vehicle_ahead.vy - 0.5 * (vehicle.ay);

            new_velocity_x = std::min(std::min(max_velocity_in_front_x, max_velocity_accel_limit_x), target_speed);
            new_velocity_x = std::min(std::min(max_velocity_in_front_x, max_velocity_accel_limit_x), target_speed);
        }
    }
    else
    {
        new_velocity_x = std::min(max_velocity_accel_limit_x, target_speed);
        new_velocity_y = std::min(max_velocity_accel_limit_y, target_speed);
    }

    new_position_x = vehicle.x + new_velocity_x + new_accel_x / 2.0;
    new_position_y = vehicle.y + new_velocity_y + new_accel_y / 2.0;
    
    Vehicle new_vehicle(vehicle);
    new_vehicle.update(new_position_x, new_position_y, new_velocity_x, new_velocity_y);

    return new_vehicle;
}