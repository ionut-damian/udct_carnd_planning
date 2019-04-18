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

void Trajectory::buildTrajectory(const Vehicle& vehicle, vector<double> anchors_x, vector<double> anchors_y, double velocity, double trajectory_length)
{
    vector<double> ptsx;
    vector<double> ptsy;

    //push point immediately before vehicle    
    ptsx.push_back(vehicle.x - cos(vehicle.theta));
    ptsy.push_back(vehicle.y - sin(vehicle.theta));

    //push current point
    ptsx.push_back(vehicle.x);
    ptsy.push_back(vehicle.y);

    //add anchors
    for (int i = 0; i < anchors_x.size(); i++)
    {
        ptsx.push_back(anchors_x[i]);
        ptsy.push_back(anchors_y[i]);
    }

    //transform to local CS
    for (int i = 0; i < ptsx.size(); i++)
    {
        double shift_x = ptsx[i] - vehicle.x;
        double shift_y = ptsy[i] - vehicle.y;

        ptsx[i] = (shift_x * cos(-vehicle.theta) - shift_y * sin(-vehicle.theta));
        ptsy[i] = (shift_x * sin(-vehicle.theta) + shift_y * cos(-vehicle.theta));
    }

    //build spline
    tk::spline spl;
    spl.set_points(ptsx, ptsy);

    //sample spline
    double last_sample_x = trajectory_length; //only sample first part of the spline
    double last_sample_y = spl(last_sample_x); //only sample first part of the spline
    double dist = distance(0, 0, last_sample_x, last_sample_y);

    double num_samples = dist / (TIME_PER_FRAME * velocity);
    double x_inc = trajectory_length / num_samples;

    //compute waypoints    
    for (int i = 1; i <= PREDICTION_WINDOW; i++)
    {
        double x_point_local = x_inc * i;
        double y_point_local = spl(x_point_local);

        double x_point = (x_point_local * cos(vehicle.theta) - y_point_local * sin(vehicle.theta));
        double y_point = (x_point_local * sin(vehicle.theta) + y_point_local * cos(vehicle.theta));

        x_point += vehicle.x;
        y_point += vehicle.y;

        waypoints_x.push_back(x_point);
        waypoints_y.push_back(y_point);
    }
}

KeepLaneTrajectory::KeepLaneTrajectory(const Vehicle& vehicle, map<int, vector<Vehicle>> &predictions)
{
    target_lane = vehicle.lane;

    vector<double> anchors_x, anchors_y;

    double spacing = 30;
    for (int i = 1; i <= 3; i++)
    {
        vector<double> point = getXY(vehicle.s + spacing * i, vehicle.lane * LANE_WIDTH + LANE_WIDTH / 2.0, Map::getInstance()->points_s, Map::getInstance()->points_x, Map::getInstance()->points_y);
        anchors_x.push_back(point[0]);
        anchors_y.push_back(point[1]);
    }

    velocity = computeVelocty(vehicle, vehicle.lane, TIME_PER_FRAME, predictions);

    buildTrajectory(vehicle, anchors_x, anchors_y, velocity, spacing);
}

PrepLaneChangeTrajectory::PrepLaneChangeTrajectory(const Vehicle& vehicle, int target_lane, map<int, vector<Vehicle>> &predictions)
{
    this->target_lane = target_lane;
    float spot_size = 30;
    float reduced_speed = 18;

    vector<double> anchors_x, anchors_y;

    //keep lane but decrease speed until we find free spot on target lane
    double spacing = 30;
    for (int i = 1; i <= 3; i++)
    {
        vector<double> point = getXY(vehicle.s + spacing * i, vehicle.lane * LANE_WIDTH + LANE_WIDTH / 2.0, Map::getInstance()->points_s, Map::getInstance()->points_x, Map::getInstance()->points_y);
        anchors_x.push_back(point[0]);
        anchors_y.push_back(point[1]);
    }

    bool targetLaneIsFree = true;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        Vehicle temp_vehicle = it->second[0];
        if (temp_vehicle.lane == target_lane && temp_vehicle.s > vehicle.s - spot_size / 2 && temp_vehicle.s < vehicle.s + spot_size / 2)
        {
            targetLaneIsFree = false;
            break;
        }
    }

    if (!targetLaneIsFree)
    {
        //reduce speed
        velocity = std::min(computeVelocty(vehicle, vehicle.lane, TIME_PER_FRAME, predictions, reduced_speed),
                            computeVelocty(vehicle, target_lane, TIME_PER_FRAME, predictions, reduced_speed));
    }
    else
    {
        //try and match velocity of car in fornt of us on the target lane
        velocity = std::min(computeVelocty(vehicle, vehicle.lane, TIME_PER_FRAME, predictions),
                            computeVelocty(vehicle, target_lane, TIME_PER_FRAME, predictions));
    }

    buildTrajectory(vehicle, anchors_x, anchors_y, velocity, spacing);
}

LaneChangeTrajectory::LaneChangeTrajectory(const Vehicle& vehicle, int target_lane, map<int, vector<Vehicle>> &predictions)
{
    this->target_lane = target_lane;

    vector<double> anchors_x, anchors_y;

    double spacing = 30;
    for (int i = 1; i <= 3; i++)
    {
        vector<double> point = getXY(vehicle.s + spacing * i, target_lane * LANE_WIDTH + LANE_WIDTH / 2.0, Map::getInstance()->points_s, Map::getInstance()->points_x, Map::getInstance()->points_y);
        anchors_x.push_back(point[0]);
        anchors_y.push_back(point[1]);
    }

    velocity = std::min(computeVelocty(vehicle, vehicle.lane, TIME_PER_FRAME, predictions),
                        computeVelocty(vehicle, target_lane, TIME_PER_FRAME, predictions));

    buildTrajectory(vehicle, anchors_x, anchors_y, velocity, 60);
}

double Trajectory::computeVelocty(const Vehicle& vehicle, int lane, double t, map<int, vector<Vehicle>> &predictions, float max_velocity)
{
    double current_velocity = vehicle.get2DVelocity();
    double max_velocity_accel_limit = current_velocity + MAX_ACC * t;
    double min_velocity_accel_limit = current_velocity - MAX_DCC * t;
    if (min_velocity_accel_limit < 0)
        min_velocity_accel_limit = 0;

    double new_velocity = max_velocity;

    Vehicle vehicle_ahead;
    if (vehicle.get_vehicle_ahead(predictions, lane, vehicle_ahead))
    {
        double dist_s = vehicle_ahead.s - vehicle.s - MIN_DIST_TO_VEHICLE;
        if (dist_s < 0)
            new_velocity = 0;
        else
        {
            double dist_thresh = 20;
            if (dist_s < dist_thresh)
            {
                new_velocity = new_velocity * (dist_s / dist_thresh) + vehicle_ahead.get2DVelocity() * (1.0 - dist_s / dist_thresh);
            }
        }
    }

    //cap velocty
    if (new_velocity > max_velocity_accel_limit)
    {
        new_velocity = max_velocity_accel_limit;
    }
    else if (new_velocity < min_velocity_accel_limit)
    {
        new_velocity = min_velocity_accel_limit;
    }
    
    if (new_velocity > max_velocity)
    {
        new_velocity = max_velocity;
    }

    return new_velocity;
}