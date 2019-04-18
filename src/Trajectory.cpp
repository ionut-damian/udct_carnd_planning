#include "Trajectory.h"
#include <algorithm>

#include "helpers.h"
#include "config.h"
#include "spline.h"


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

    vector<double> anchors_x, anchors_y;

    //push point immediately before vehicle    
    anchors_x.push_back(vehicle.x - cos(vehicle.theta));
    anchors_y.push_back(vehicle.y - sin(vehicle.theta));

    //push current point
    anchors_x.push_back(vehicle.x);
    anchors_y.push_back(vehicle.y);

    //push 3 more points, once every second
    double spacing = 30; //meters
    for (int i = 1; i <= 3; i++)
    {
        vector<double> point = getXY(vehicle.s + spacing * i, vehicle.lane * LANE_WIDTH + LANE_WIDTH / 2.0, Map::getInstance()->points_s, Map::getInstance()->points_x, Map::getInstance()->points_y);
        anchors_x.push_back(point[0]);
        anchors_y.push_back(point[1]);
    }  

    //build spline
    tk::spline spl;
    spl.set_points(anchors_x, anchors_y);

    //sample spline
    double last_sample_x = vehicle.x + spacing; //only sample first part of the spline
    double last_sample_y = spl(last_sample_x); //only sample first part of the spline
    double dist = distance(vehicle.x, vehicle.y, last_sample_x, last_sample_y);

    //double new_velocity = computeVelocty(vehicle, vehicle.lane, TIME_PER_FRAME, )
    double num_samples = dist / (TIME_PER_FRAME * target_speed);
    double x_inc = spacing / num_samples;
    
    //compute waypoints    
    for (int i = 1; i <= PREDICTION_WINDOW; i++)
    {      
        double x_point = vehicle.x + x_inc * i;
        double y_point = spl(x_point);

        waypoints_x.push_back(x_point);
        waypoints_y.push_back(y_point);
    }
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

    for (int i = 1; i < PREDICTION_WINDOW; i++)
        waypoints.push_back(get_kinematics(waypoints.back(), waypoints.back().lane, TIME_PER_FRAME, predictions, max_acceleration, target_speed));


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

    for (int i = 1; i < PREDICTION_WINDOW; i++)
        waypoints.push_back(get_kinematics(waypoints.back(), waypoints.back().lane, TIME_PER_FRAME, predictions, max_acceleration, target_speed));
}

double Trajectory::computeVelocty(Vehicle& vehicle, int lane, double t, map<int, vector<Vehicle>> &predictions, float max_acceleration, float target_speed)
{
    double current_velocity = vehicle.get2DVelocity();
    double max_velocity_accel_limit = current_velocity + max_acceleration * t;
    double min_velocity_accel_limit = current_velocity - max_acceleration * t;
    if (min_velocity_accel_limit < 0)
        min_velocity_accel_limit = 0;

    double new_velocity = target_speed;

    Vehicle vehicle_ahead;
    if (vehicle.get_vehicle_ahead(predictions, lane, vehicle_ahead))
    {
        double dist_s = vehicle_ahead.s - vehicle.s - vehicle.preferred_buffer;
        if (dist_s < 0)
            new_velocity = 0;

        double dist_thresh = 20;
        if (dist_s < dist_thresh)
        {
            new_velocity = new_velocity * (dist_s / dist_thresh) + vehicle_ahead.get2DVelocity() * (1.0 - dist_s / dist_thresh);
        }
    }

    //cap velocty
    if (new_velocity > max_velocity_accel_limit)
    {
        new_velocity = max_velocity_accel_limit;
        if (new_velocity > target_speed)
        {
            new_velocity = target_speed;
        }
    }
    else if (new_velocity < min_velocity_accel_limit)
    {
        new_velocity = min_velocity_accel_limit;
        if (new_velocity > target_speed)
        {
            new_velocity = target_speed;
        }
    }

    return new_velocity;
}

Vehicle Trajectory::get_kinematics(Vehicle& vehicle, int lane, double t, map<int, vector<Vehicle>> &predictions, float max_acceleration, float target_speed)
{
    double current_velocity = vehicle.get2DVelocity();
    double max_velocity_accel_limit = current_velocity + max_acceleration *t;
    double min_velocity_accel_limit = current_velocity - max_acceleration *t;
    if (min_velocity_accel_limit < 0)
        min_velocity_accel_limit = 0;

    double new_velocity = target_speed;    

    Vehicle vehicle_ahead;
    if (vehicle.get_vehicle_ahead(predictions, lane, vehicle_ahead))
    {
        double dist_s = vehicle_ahead.s - vehicle.s - vehicle.preferred_buffer;
        if (dist_s < 0)
            new_velocity = 0;

        double dist_thresh = 20;
        if (dist_s < dist_thresh)
        {
            new_velocity = new_velocity * (dist_s / dist_thresh) + vehicle_ahead.get2DVelocity() * (1.0 - dist_s / dist_thresh);
        }
    }

    //cap velocty
    if (new_velocity > max_velocity_accel_limit)
    {
        new_velocity = max_velocity_accel_limit;
        if (new_velocity > target_speed)
        {
            new_velocity = target_speed;
        }
    }
    else if (new_velocity < min_velocity_accel_limit)
    {
        new_velocity = min_velocity_accel_limit;
        if (new_velocity > target_speed)
        {
            new_velocity = target_speed;
        }
    }

    double new_accel = (new_velocity - current_velocity) / t;
    double new_position_s = vehicle.s + current_velocity * t + new_accel * t*t / 2.0;

    //float new_velocity_x = new_velocity * cos(vehicle.theta);
    //float new_velocity_y = new_velocity * sin(vehicle.theta);

    double new_d = lane * LANE_WIDTH + LANE_WIDTH / 2.0;
    vector<double> new_position = getXY(new_position_s, new_d, Map::getInstance()->points_s, Map::getInstance()->points_x, Map::getInstance()->points_y);
    
    Vehicle new_vehicle;

    new_vehicle.x = new_position[0];
    new_vehicle.y = new_position[1];
    new_vehicle.s = new_position_s;
    new_vehicle.d = new_d;
    new_vehicle.lane = lane;

    new_vehicle.vx = (new_vehicle.x - vehicle.x) / t;
    new_vehicle.vy = (new_vehicle.y - vehicle.y) / t;

    new_vehicle.ax = (new_vehicle.vx - vehicle.vx) / t;
    new_vehicle.ay = (new_vehicle.vy - vehicle.vy) / t;

    new_vehicle.theta = atan2(new_vehicle.y - vehicle.y, new_vehicle.x - vehicle.x);
    //if (new_vehicle.theta < 0)
    //    new_vehicle.theta = 360.0 - abs(new_vehicle.theta);
    
    //Vehicle new_vehicle(vehicle);
    //new_vehicle.update(new_position[0], new_position[1]);

    return new_vehicle;
}