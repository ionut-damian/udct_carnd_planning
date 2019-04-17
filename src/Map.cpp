#include "Map.h"

Map* Map::instance = NULL;

Map* Map::getInstance()
{
    if (instance == NULL)
    {
        instance = new Map();
    }

    return instance;
}

Map::Map()
{
}
