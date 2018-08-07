#include "map_server/map_reps/map_reps_ros.h"
#include <typeinfo>

MapServerROS::MapServerROS()
{
occ_service = n.advertiseService("static_occ_grid", &MapServerROS::OccMapCallback, this);
gridmap_service = n.advertiseService("static_gridmap", &MapServerROS::GridMapCallback, this);

// Latched publishers for data
occmap_pub = n.advertise<nav_msgs::OccupancyGrid>("occmap", 1, true);
gridmap_pub = n.advertise<grid_map_msgs::GridMap>("gridmap", 1, true);

} 

// Functions to publish map Message

void MapServerROS::publishMap(nav_msgs::OccupancyGrid map)
{
    occmap_pub.publish(map);
}

void MapServerROS::publishMap(grid_map_msgs::GridMap map)
{
    gridmap_pub.publish(map);
}

void MapServerROS::publishMap(boost::any map)
{
    //std::type_info &ti = map.type();

    if (map.type()==typeid(occmap_resp_.map))
        occmap_pub.publish(boost::any_cast<nav_msgs::OccupancyGrid>(map));
    else if (map.type()==typeid(gridmap_resp_.map))
         gridmap_pub.publish(boost::any_cast<grid_map_msgs::GridMap>(map));
}
// Functions to set service response

void MapServerROS::setResponse(nav_msgs::OccupancyGrid map)
{
    occmap_resp_.map = map;
}
void MapServerROS::setResponse(grid_map_msgs::GridMap map)
{

    gridmap_resp_.map = map;
}
void MapServerROS::setResponse(boost::any map)
{
   // std::type_info &ti = map.type();


    //map_type = &map.type();

    if (map.type()==typeid(occmap_resp_.map))
    {
        occmap_resp_.map = boost::any_cast<nav_msgs::OccupancyGrid>(map);
        type_names[std::type_index(typeid(occmap_resp_.map))] = "occupancy";
        ROS_INFO("%s",type_names[std::type_index(typeid(occmap_resp_.map))].c_str());
    }

    else if (map.type()==typeid(gridmap_resp_.map))
    {
        gridmap_resp_.map = boost::any_cast<grid_map_msgs::GridMap>(map);
        type_names[std::type_index(typeid(gridmap_resp_.map))] = "gridmap";
        ROS_INFO("%s",type_names[std::type_index(typeid(gridmap_resp_.map))].c_str());     
    }   
}


// Callbacks


bool MapServerROS::OccMapCallback(nav_msgs::GetMap::Request  &req,
                nav_msgs::GetMap::Response &res )
{
    //TODO: check if map type exists, if not, convert from available map type
    if (type_names[std::type_index(typeid(occmap_resp_.map))]!="occupancy")
    {
        boost::any message;
        //MapConverter * Converter = new MapConverter();
        ROS_INFO("Converting...");
        grid_map::GridMap grid({"occupancy"});
        Converter.fromMessage(gridmap_resp_.map, grid);
        message = Converter.convertMap(grid, occmap_resp_.map,"occupancy", 0.65,0.196);
        occmap_resp_.map = boost::any_cast<nav_msgs::OccupancyGrid>(message);
        occmap_pub.publish(occmap_resp_.map);
    }
    res = occmap_resp_;    
    ROS_INFO("Sending map");

    return true;
}

bool MapServerROS::GridMapCallback(grid_map_msgs::GetGridMap::Request  &req,
                grid_map_msgs::GetGridMap::Response &res)
{

    //TODO: check if map type exists, if not, convert from available map type


    res = gridmap_resp_;
    ROS_INFO("Sending map");

    return true;
}

