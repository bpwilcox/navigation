#ifndef ENV_MODEL_OCCUPANCY_GRID_ENV_H
#define ENV_MODEL_OCCUPANCY_GRID_ENV_H

#include "env_model/base_env_model.h"
#include <nav_msgs/OccupancyGrid.h>
#include <unordered_map>
#include <string>

class OccupancyGridEnv : public BaseEnvModel
{
    private:
        double resolution;
        double size_x;
        double size_y;
        double origin_x;
        double origin_y;
        signed char * data;

        struct layer_t
        {
            std::string name;
            signed char * data; 
        }; 

        struct Layers_t
        {        
            std::unordered_map<std::string, int> layer_id;  
            layer_t *layer; 
            layer_t operator() (std::string name)
            {   
                return layer[layer_id[name]];
            }
        };

        std::unordered_map<std::string, layer_t> MapLayers;  



    public:
        
        


        OccupancyGridEnv(){}
        OccupancyGridEnv(std::string layer, nav_msgs::OccupancyGrid map);

        // MAP SPECIFIC //

        nav_msgs::OccupancyGrid map;
        Layers_t LayerMap;
        void convertMap(std::string layer, nav_msgs::OccupancyGrid map);
        int MapToDataIndex(int i, int j);
        void addMapLayer(std::string layer);
        void addMapLayer(std::string layer, nav_msgs::OccupancyGrid map);

        // INTERFACE FUNCTIONS //

        // get map coordinates from cell index
        double getPosX(int i);
        double getPosY(int j);

        // get map cell index from map coordinates
        int getCellX(double x);
        int getCellY(double y);

        // perform raytracing
        double getRayTrace(double x, double y, double a, double max_range);

        //Check if map is valid 
        bool isValid(int i, int j);

        //get map value by index
        float getValueAt(std::string layer, int i, int j);

        // get map value by position 
        float getValueAtPos(std::string layer, double x, double y);
        
        //Update the map layer specified
        void updateMap(std::string layer);
        
        ~OccupancyGridEnv(){}



};

#endif