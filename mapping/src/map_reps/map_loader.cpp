#include "map_server/map_loader.h"

//Factory Class for loading new map types

BaseMapLoader* MapLoader::createMap(std::string mapType)
{
    if (mapType=="occupancy")
    {
        /*
        BaseMapLoader *Base = new OccGridLoader;
        OccGridLoader *Occ = dynamic_cast<OccGridLoader*>(Base);

        return Occ;
        */
        return new OccGridLoader;
    }
    else if (mapType=="gridmap")
    {
        /*
        BaseMapLoader *Base = new GridMapLoader;
        GridMapLoader *Grid = dynamic_cast<GridMapLoader*>(Base);

        return Grid;
        */

        return new GridMapLoader;
    

    }
}