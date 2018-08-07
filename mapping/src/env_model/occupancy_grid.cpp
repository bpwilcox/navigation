#include "env_model/occupancy_grid.h"

    
//For now, implement calculations as previous
        
OccupancyGridEnv::OccupancyGridEnv(std::string layer, nav_msgs::OccupancyGrid map) 
{
    resolution = map.info.resolution;
    size_x = map.info.width;
    size_y = map.info.height;
    //origin here is wrt [0,0] cell, or real world origin
    origin_x = map.info.origin.position.x + (size_x / 2) * resolution;
    origin_y = map.info.origin.position.y + (size_y / 2) * resolution;
    addMapLayer(layer,map);
    //data = map.data.data();
}

// MAP SPECIFIC //

void OccupancyGridEnv::convertMap(std::string layer, nav_msgs::OccupancyGrid map) 
{
    resolution = map.info.resolution;
    size_x = map.info.width;
    size_y = map.info.height;
    //origin here is wrt [0,0] cell, or real world origin
    origin_x = map.info.origin.position.x + (size_x / 2) * resolution;
    origin_y = map.info.origin.position.y + (size_y / 2) * resolution;
    addMapLayer(layer,map);
    //data = map.data.data();
}

int OccupancyGridEnv::MapToDataIndex(int i, int j) {return i + j * size_x;}

void OccupancyGridEnv::addMapLayer(std::string layer)
{
    layer_t new_layer;
    new_layer.name = layer;
    MapLayers[layer] = new_layer;

}
/*
void OccupancyGridEnv::addMapLayer(std::string layer)
{
    layer_id[layer] = 0;
    layer_t new_layer;
    new_layer.name = layer;

}
*/
void OccupancyGridEnv::addMapLayer(std::string layer, nav_msgs::OccupancyGrid map)
{
    layer_t new_layer;
    new_layer.name = layer;
    new_layer.data = map.data.data();
    MapLayers[layer] = new_layer;

}


// INTERFACE FUNCTIONS //

// get map coordinates from cell index
double OccupancyGridEnv::getPosX(int i) {return origin_x + (i-size_x / 2) * resolution;}

double OccupancyGridEnv::getPosY(int j) {return origin_y + (j-size_y / 2) * resolution;}

// get map cell index from map coordinates
int OccupancyGridEnv::getCellX(double x) {return floor((x - origin_x) / resolution + 0.5) + size_x / 2;}
int OccupancyGridEnv::getCellY(double y) {return floor((y - origin_y) / resolution + 0.5) + size_y / 2;}

// perform raytracing
double OccupancyGridEnv::getRayTrace(double x, double y, double a, double max_range) {}

//Check if map is valid 
bool OccupancyGridEnv::isValid(int i, int j) {return (i >= 0) && (i < size_x) && (j >= 0) && (j < size_y);}

//get map value by index
float OccupancyGridEnv::getValueAt(std::string layer, int i, int j) 
{
    int index;
    float value;
    index = MapToDataIndex(i,j);
    //value = (float) (LayerMap(layer).data[index]-'0');
    value = (float) (MapLayers[layer].data[index]-'0');

    return value;
}

// get map value by position 
float OccupancyGridEnv::getValueAtPos(std::string layer, double x, double y) 
{
    int i, j;
    i = getCellX(x);
    j = getCellY(y);
    return getValueAt(layer,i,j);

}

//Update the map layer specified
void OccupancyGridEnv::updateMap(std::string layer) {}
        

