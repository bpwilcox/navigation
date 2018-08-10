#ifndef ENV_MODEL_OCCUPANCY_GRID_ENV_H
#define ENV_MODEL_OCCUPANCY_GRID_ENV_H

#include "env_model/base_env_model.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <unordered_map>
#include <string>
#include "ros/ros.h"
#include "env_model/occ_grid_layer.h"

class OccupancyGridEnv : public BaseEnvModel
{
    private:
        double resolution;
        int size_x;
        int size_y;
        double origin_x;
        double origin_y;

        //This map is a placeholder, it can be overwritten via getMap 
        nav_msgs::OccupancyGrid map;
        nav_msgs::GetMap occ_srv; 

        //This is our container for map layers 
        std::unordered_map<std::string, layer_t> MapLayers;  

    public:
        
        

        //Should constructor initialize map? 
        OccupancyGridEnv(){}

        // MAP SPECIFIC //
        
        int MapToDataIndex(int i, int j);

        // INTERFACE FUNCTIONS //

        // get map coordinates from cell index
        double getPosX(int i);
        double getPosY(int j);

        // get map cell index from map coordinates
        int getCellX(double x);
        int getCellY(double y);

        // perform raytracing
        double getRayTrace(std::string occ_layer, double ox, double oy, double oa, double max_range);

        //Check if map is valid 
        bool isValid(int i, int j);

        //get map value by index
        float getValueAt(std::string layer, int i, int j);

        // get map value by position 
        float getValueAtPos(std::string layer, double x, double y);
        
        // grab map from map server
        bool getMap(std::string mapname);

        // initialize metadata and layer from map
        bool initializeMapFromServer(std::string layer, std::string mapname);

        // add an empty map layer
        void addMapLayer(std::string layer);

        // add a specified map to layer (may decide to reject map if doesn't match the metadata)
        void addMapLayer(std::string layer,std::string mapname);

        //Update the map layer specified
        void updateMap(std::string layer);
        
        void setLayerData(std::string layer);

        void setLayerDataOcc(std::string layer, layer_t mylayer);
        
        ~OccupancyGridEnv(){}



};

#endif