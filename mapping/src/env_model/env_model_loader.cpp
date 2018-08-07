#include "env_model/env_models.h"

class EnvModelLoader 
{
    public:
        static EnvModelLoader* createEnv(std::string EnvType)
        {
            if (EnvType=="occupancy")
            {
                return new OccupancyGridEnv;
            }
            else if (EnvType=="gridmap")
            {
                return new GridMapEnv;
            
            }                           
        }

};

