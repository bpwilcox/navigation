#include "map_server/map_reps/map_reps.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include <grid_map_msgs/GetGridMap.h>
#include "map_server/map_converter.h"
#include <typeinfo>
#include <typeindex>
#include <unordered_map>
#include <string>
class MapServerROS
{
    public:

        MapServerROS();

        //std::type_info map_type;

        void publishMap(nav_msgs::OccupancyGrid map);

        void publishMap(grid_map_msgs::GridMap map);

        void publishMap(boost::any map);

        void setResponse(nav_msgs::OccupancyGrid map);

        void setResponse(grid_map_msgs::GridMap map);

        void setResponse(boost::any map);
    private:

        //const std::type_info * const map_type = &typeid(int);
        std::unordered_map<std::type_index, std::string> type_names;       

        ros::NodeHandle n;   
        //occupancy grid
        ros::Publisher occmap_pub;
        ros::ServiceServer occ_service;

        // gridmap
        ros::Publisher gridmap_pub;
        ros::ServiceServer gridmap_service;

        bool OccMapCallback(nav_msgs::GetMap::Request  &req,
                        nav_msgs::GetMap::Response &res);

        bool GridMapCallback(grid_map_msgs::GetGridMap::Request  &req,
                        grid_map_msgs::GetGridMap::Response &res);

        nav_msgs::GetMap::Response occmap_resp_;
        grid_map_msgs::GetGridMap::Response gridmap_resp_;
        MapConverter Converter;
};