#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "helpers.h"
#include "Behavior.h"
#include "Environment.h"
#include "Map.h"
#include "config.h"
#include "spline.h"

#define MAX_OVERLAP_PREV_PATH 50u

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    Map* map = Map::getInstance();

    // Waypoint map to read from  
#ifdef _MSC_VER    
    string map_file_ = "../../data/highway_map.csv";
#else
    string map_file_ = "../data/highway_map.csv";
#endif
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map->points_x.push_back(x);
        map->points_y.push_back(y);
        map->points_s.push_back(s);
        map->points_dx.push_back(d_x);
        map->points_dy.push_back(d_y);
    }
           
    Behavior planner(3, 20, 10, 0, 10);  
    planner.ego.lane = 1;

    Environment environment;

#ifdef _MSC_VER    
    h.onMessage([&planner, &environment](uWS::WebSocket<uWS::SERVER>* ws, char *data, size_t length, uWS::OpCode opCode)
    {
#else
    h.onMessage([&planner, &environment](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
    {
#endif
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {

            auto s = hasData(data);

            if (s != "")
            {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = deg2rad(j[1]["yaw"]);
                    double car_speed = (j[1]["speed"] * 1609.34) / 3600.0;

                    double car_vx = cos(car_yaw) * car_speed;
                    double car_vy = sin(car_yaw) * car_speed;
                                       
                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values 
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    int path_size = std::min(previous_path_x.size(), MAX_OVERLAP_PREV_PATH);
                    for (int i = 0; i < path_size; ++i)
                    {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    if (path_size > 1)
                    {
                        double path_last_x = previous_path_x[path_size - 1];
                        double path_last_y = previous_path_y[path_size - 1];

                        double path_prevlast_x = previous_path_x[path_size - 2];
                        double path_prevlast_y = previous_path_y[path_size - 2];

                        //car_vx = (car_x - previous_path_x[path_size - 2]) / TIME_PER_FRAME;
                        //car_vy = (car_y - previous_path_y[path_size - 2]) / TIME_PER_FRAME;
                        car_vx = planner.ego.vx;
                        car_vy = planner.ego.vy;

                        double car_yaw_frompath = atan2(path_last_y - path_prevlast_y, path_last_x - path_prevlast_x);
                        //if (car_yaw < 0)
                        //    car_yaw = 360 - abs(car_yaw);

                        planner.ego.update(path_last_x, path_last_y, car_vx, car_vy, car_s, car_d, 0,0, car_yaw_frompath);
                    }
                    else
                    {
                        planner.ego.update(car_x, car_y, car_vx, car_vy, car_s, car_d, 0, 0, car_yaw);
                    }

                    // Sensor Fusion Data, a list of all other cars on the same side 
                    //   of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];
                    for (int i = 0; i < sensor_fusion.size(); i++)
                    {
                        environment.updateVehicle(
                            sensor_fusion[i][0], //id
                            sensor_fusion[i][1], //x
                            sensor_fusion[i][2], //y
                            sensor_fusion[i][3], //vx
                            sensor_fusion[i][4], //vy
                            sensor_fusion[i][5], //s
                            sensor_fusion[i][6]);//d
                    }

                    //process

                    //vector<double> anchors_x, anchors_y;                    

                    ////push point immediately before vehicle    
                    //anchors_x.push_back(planner.ego.x - cos(planner.ego.theta));
                    //anchors_y.push_back(planner.ego.y - sin(planner.ego.theta));

                    ////push current point
                    //anchors_x.push_back(planner.ego.x);
                    //anchors_y.push_back(planner.ego.y);


                    ////push 3 more points, once every second
                    //double spacing = 30; //meters
                    //for (int i = 1; i <= 3; i++)
                    //{
                    //    vector<double> point = getXY(planner.ego.s + spacing * i, planner.ego.lane * LANE_WIDTH + LANE_WIDTH / 2.0, Map::getInstance()->points_s, Map::getInstance()->points_x, Map::getInstance()->points_y);
                    //    anchors_x.push_back(point[0]);
                    //    anchors_y.push_back(point[1]);
                    //}

                    ////transform to local CS
                    //for (int i = 0; i < anchors_x.size(); i++)
                    //{
                    //    double shift_x = anchors_x[i] - planner.ego.x;
                    //    double shift_y = anchors_y[i] - planner.ego.y;

                    //    anchors_x[i] = (shift_x * cos(-planner.ego.theta) - shift_y * sin(-planner.ego.theta));
                    //    anchors_y[i] = (shift_x * sin(-planner.ego.theta) + shift_y * cos(-planner.ego.theta));
                    //}

                    ////build spline
                    //tk::spline spl;
                    //spl.set_points(anchors_x, anchors_y);

                    ////sample spline
                    //double last_sample_x = spacing; //only sample first part of the spline
                    //double last_sample_y = spl(last_sample_x); //only sample first part of the spline
                    //double dist = distance(0, 0, last_sample_x, last_sample_y);

                    ////double new_velocity = computeVelocty(vehicle, vehicle.lane, TIME_PER_FRAME, )
                    //double num_samples = dist / (TIME_PER_FRAME * 20);
                    //double x_inc = spacing / num_samples;

                    ////compute waypoints    
                    //for (int i = 0; i < PREDICTION_WINDOW - previous_path_x.size(); i++)
                    //{
                    //    double x_point = x_inc * i;
                    //    double y_point = spl(x_point);

                    //    double x_point_local = x_point;
                    //    double y_point_local = y_point;

                    //    x_point = (x_point_local * cos(planner.ego.theta) - y_point_local * sin(planner.ego.theta));
                    //    y_point = (x_point_local * sin(planner.ego.theta) + y_point_local * cos(planner.ego.theta));

                    //    x_point += planner.ego.x;
                    //    y_point += planner.ego.y;

                    //    next_x_vals.push_back(x_point);
                    //    next_y_vals.push_back(y_point);
                    //}
                    
                    
                    Trajectory* traj = planner.choose_next_state(environment);
                    //planner.ego.realize_next_state(traj->waypoints[0]);


                    //Trajectory* traj = new KeepLaneTrajectory(planner.ego, std::map<int, vector<Vehicle>>(), 10, 20);
                    for (int i = 0; i < PREDICTION_WINDOW - previous_path_x.size(); i++)
                    {
                        next_x_vals.push_back(traj->waypoints_x[i]);
                        next_y_vals.push_back(traj->waypoints_y[i]);
                    }

                    delete traj;


                    //printf("out: x=%.2f, y=%.2f, yaw_in=%.2f, yaw=%.2f, v_in=%.2f, v_in_c=%.2f, v=%.2f\n", 
                    //    traj->waypoints[0].x, traj->waypoints[0].y, 
                    //    rad2deg(car_yaw), rad2deg(traj->waypoints[0].theta), 
                    //    car_speed, sqrt(pow(car_vx,2) + pow(car_vy, 2)), traj->waypoints[0].get2DVelocity());
                    //printf("out: x=%.2f, y=%.2f, v=%.2f\n", traj->waypoints[10].x, traj->waypoints[10].y, traj->waypoints[10].get2DVelocity());

                    json msgJson;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

#ifdef _MSC_VER      
                    ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT); //vs
#else
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); //gcc
#endif
                }  // end "telemetry" if
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
#ifdef _MSC_VER      
                ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT); //vs
#else
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); //gcc
#endif
            }
        }  // end websocket if
    }); // end h.onMessage

#ifdef _MSC_VER      
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) //vs
#else
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) //gcc
#endif
    {
        std::cout << "Connected!!!" << std::endl;
    });

#ifdef _MSC_VER      
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code, char *message, size_t length) //vs
    {
        ws->close();
        std::cout << "Disconnected" << std::endl;
    });
#else
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) //gcc
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
#endif

    int port = 4567;
    auto host = "127.0.0.1";
    if (h.listen(host, port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}