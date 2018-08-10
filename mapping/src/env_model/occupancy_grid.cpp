#include "env_model/occupancy_grid.h"
#include "ros/ros.h"
#include <math.h>    
#include <typeinfo>
#include "utilities/cacheddistmap.h"
//For now, implement calculations as previous
        

// MAP SPECIFIC //

void OccupancyGridEnv::addMapLayer(std::string layer) 
{
    layer_t new_layer;
    new_layer.name = layer;
    MapLayers[layer] = new_layer;

    ROS_INFO("Empty Layer '%s' added", layer.c_str());

}

void OccupancyGridEnv::addMapLayer(std::string layer, std::string mapname) 
{
    getMap(mapname);
    layer_t new_layer;
    new_layer.name = layer;
    new_layer.data = map.data.data();
    new_layer.data_type = typeid(new_layer.data).name();
    MapLayers[layer] = new_layer;

    ROS_INFO("Layer '%s' added with map '%s'", layer.c_str(), mapname.c_str());
    ROS_INFO("data type is '%s'", new_layer.data_type.c_str());

}

int OccupancyGridEnv::MapToDataIndex(int i, int j) {return i + j * size_x;}


// INTERFACE FUNCTIONS //

// get map coordinates from cell index
double OccupancyGridEnv::getPosX(int i) {return origin_x + (i-size_x / 2) * resolution;}

double OccupancyGridEnv::getPosY(int j) {return origin_y + (j-size_y / 2) * resolution;}

// get map cell index from map coordinates
int OccupancyGridEnv::getCellX(double x) {return floor((x - origin_x) / resolution + 0.5) + size_x / 2;}

int OccupancyGridEnv::getCellY(double y) {return floor((y - origin_y) / resolution + 0.5) + size_y / 2;}



//Check if map is valid 
bool OccupancyGridEnv::isValid(int i, int j) {return (i >= 0) && (i < size_x) && (j >= 0) && (j < size_y);}

//get map value by index
float OccupancyGridEnv::getValueAt(std::string layer, int i, int j) 
{
    int index;
    float value;
    index = MapToDataIndex(i,j);
    //value = (float) (LayerMap(layer).data[index]-'0');
    //value = (float) (MapLayers[layer].data[index] - '0');

    if(typeid(MapLayers[layer].data).name()==MapLayers[layer].data_type)
        value = (float) (MapLayers[layer].data[index]);
    else
        value = (float) (MapLayers[layer].ddata[index]);


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

bool OccupancyGridEnv::getMap(std::string mapname)
{
    if (ros::service::call(mapname,occ_srv))
    {
        ROS_INFO("Occupancy Grid Environment Created!");
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    map = occ_srv.response.map;
    return 0;


}

bool OccupancyGridEnv::initializeMapFromServer(std::string layer, std::string mapname)
{
    addMapLayer(layer, mapname);
    resolution = map.info.resolution;
    size_x = map.info.width;
    size_y = map.info.height;
    origin_x = map.info.origin.position.x + (size_x / 2) * resolution;
    origin_y = map.info.origin.position.y + (size_y / 2) * resolution;


    addMapLayer("distance");
    CachedDistanceMap * cdm = new CachedDistanceMap(resolution, 100.0, size_x, size_y);
    cdm->map_update_cspace(MapLayers[layer],size_x, size_y, resolution, 100.0);
    setLayerDataOcc("distance", cdm->cmap);
}
void OccupancyGridEnv::setLayerData(std::string layer)
{


}

void OccupancyGridEnv::setLayerDataOcc(std::string layer, layer_t mylayer)
{
    MapLayers[layer].ddata = mylayer.ddata;
    MapLayers[layer].data_type = typeid(mylayer.ddata).name();
    ROS_INFO("Added Data to Layer '%s'", layer.c_str());
}

//Update the map layer specified
void OccupancyGridEnv::updateMap(std::string layer) 
{
}
        
// perform raytracing
double OccupancyGridEnv::getRayTrace(std::string occ_layer, double ox, double oy, double oa, double max_range) 
{
    // Bresenham raytracing
    int x0,x1,y0,y1;
    int x,y;
    
    int xstep, ystep;
    char steep;
    int tmp;
    int deltax, deltay, error, deltaerr;

    x0 = getCellX(ox);
    y0 = getCellY(oy);


    x1 = getCellX(ox + max_range * cos(oa));
    y1 = getCellY(oy + max_range * sin(oa));

    if(abs(y1-y0) > abs(x1-x0))
        steep = 1;
    else
        steep = 0;

    if(steep)
    {
        tmp = x0;
        x0 = y0;
        y0 = tmp;

        tmp = x1;
        x1 = y1;
        y1 = tmp;
    }

    deltax = abs(x1-x0);
    deltay = abs(y1-y0);
    error = 0;
    deltaerr = deltay;

    x = x0;
    y = y0;

    if(x0 < x1)
        xstep = 1;
    else
        xstep = -1;
    if(y0 < y1)
        ystep = 1;
    else
        ystep = -1;

    if(steep)
    {
    if(!isValid(y,x) || getValueAt(occ_layer,y,x) > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * resolution;
    }
    else
    {
    if(!isValid(x,y) || getValueAt(occ_layer,x,y) > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * resolution;
    }

    while(x != (x1 + xstep * 1))
    {
        x += xstep;
        error += deltaerr;
        if(2*error >= deltax)
        {
            y += ystep;
            error -= deltax;
        }

    if(steep)
    {
        if(!isValid(y,x) || getValueAt(occ_layer,y,x) > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * resolution;
    }
    else
    {
        if(!isValid(x,y) || getValueAt(occ_layer,x,y) > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * resolution;
    }
    }
    return max_range;

}
