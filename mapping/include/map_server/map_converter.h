#ifndef MAP_SERVER_MAP_CONVERTER_H
#define MAP_SERVER_MAP_CONVERTER_H

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/any.hpp>

using namespace grid_map;

//enum MapMode {TRINARY, SCALE, RAW};

/*
    This class is a wrapper for conversions between different map representations
    Map Conversions currently available are:
        Occupancy Grid --> Grid Map
        Grid Map --> Occupancy Grid

*/
class MapConverter: public GridMapRosConverter
{
public:
    MapConverter(){};



    float dataMin;
    float dataMax;
    boost::any converted;


    virtual ~MapConverter(){};

    //Convert from Occupancy Grid message to GridMap Object
    boost::any convertMap(const nav_msgs::OccupancyGrid occ, GridMap grid, const std::string& layer);

    //Convert from GridMap object to Occupancy Grid message
    //map mode default is TRINARY to convert
    boost::any convertMap(GridMap grid, nav_msgs::OccupancyGrid occ, const std::string& layer, double occ_thresh, double free_thresh);


};

#endif