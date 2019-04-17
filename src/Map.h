#pragma once
#ifndef MAP_H
#define MAP_H

#include <vector>

class Map
{
public:
    static Map* getInstance();

    std::vector<double> points_x;
    std::vector<double> points_y;
    std::vector<double> points_s;
    std::vector<double> points_dx;
    std::vector<double> points_dy;

private:
    Map();
    static Map* instance;
};

#endif