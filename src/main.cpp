#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "helpers.h"
#include "Behavior.h"
#include "Map.h"

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
           
    Behavior planner(3, 50, 10, 0, 10);    

#ifdef _MSC_VER    
    h.onMessage([](uWS::WebSocket<uWS::SERVER>* ws, char *data, size_t length, uWS::OpCode opCode)
    {
#else
    h.onMessage([](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
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
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values 
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];



                    // Sensor Fusion Data, a list of all other cars on the same side 
                    //   of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    vector<vector<double>> vehicles;
                    for (int i = 0; i < sensor_fusion.size(); i++)
                    {
                        vehicles.push_back(vector<double>());
                        for (int j = 0; j < sensor_fusion[i].size(); j++)
                        {
                            vehicles.back().push_back(sensor_fusion[i][j]);
                        }
                    }

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    /**
                     * TODO: define a path made up of (x,y) points that the car will visit
                     *   sequentially every .02 seconds
                     */

                    //TEST
                    double dist_inc = 0.5;
                    for (int i = 0; i < 50; ++i)
                    {
                        next_x_vals.push_back(car_x + (dist_inc*i)*cos(deg2rad(car_yaw)));
                        next_y_vals.push_back(car_y + (dist_inc*i)*sin(deg2rad(car_yaw)));
                    }


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