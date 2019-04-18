#include "Map.h"

Map* Map::instance = 0;

Map* Map::getInstance()
{
    if (instance == 0)
    {
        instance = new Map();
    }

    return instance;
}

Map::Map()
{
}
