#ifndef MAP_SERVER_MAP_REPS_OCCUPANCY_GRID_LOADER_H
#define MAP_SERVER_MAP_REPS_OCCUPANCY_GRID_LOADER_H

#include "map_server/base_map_loader.h"
#include <nav_msgs/OccupancyGrid.h>

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"

using namespace nav_msgs;

class OccGridLoader: public BaseMapLoader
{
    public:
        OccupancyGrid map_msg;
        //Map<OccupancyGrid> map_msg;

        OccGridLoader(){}
        
        void loadMapInfoFromFile(std::string fname);
        
        void loadMapFromFile(std::string mapfname);

        ~OccGridLoader(){}
};

#endif