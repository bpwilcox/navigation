#include "env_model/grid_map.h"
#include "ros/ros.h"
#include <math.h>    

//For now, implement calculations as previous
        

// MAP SPECIFIC //


// INTERFACE FUNCTIONS //

// get map coordinates from cell index
double GridMapEnv::getPosX(int i) {}

double GridMapEnv::getPosY(int j) {}

// get map cell index from map coordinates
int GridMapEnv::getCellX(double x) {}

int GridMapEnv::getCellY(double y) {}



//Check if map is valid 
bool GridMapEnv::isValid(int i, int j) {}

//get map value by index
float GridMapEnv::getValueAt(std::string layer, int i, int j) 
{
   
}

// get map value by position 
float GridMapEnv::getValueAtPos(std::string layer, double x, double y) 
{
    

}

bool GridMapEnv::getMap(std::string mapname)
{
    if (ros::service::call(mapname,map_msg))
    {
        ROS_INFO("GridMap Environment Created!");
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    GridMapRosConverter::fromMessage(map_msg,map);

    return 0;

}

bool GridMapEnv::initializeMapFromServer(std::string layer, std::string mapname)
{
   
}

void GridMapEnv::addMapLayer(std::string layer) 
{


}

void GridMapEnv::addMapLayer(std::string layer, std::string mapname) 
{

}
//Update the map layer specified
void GridMapEnv::updateMap(std::string layer) {}
        
// perform raytracing
double GridMapEnv::getRayTrace(std::string occ_layer, double ox, double oy, double oa, double max_range) 
{
   

}
