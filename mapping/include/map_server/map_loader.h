#ifndef MAP_SERVER_MAP_LOADER_H
#define MAP_SERVER_MAP_LOADER_H

#include "map_server/map_reps/map_reps.h"


class MapLoader 
{
    public:
        static BaseMapLoader* createMap(std::string mapType);

};

#endif