#include "env_model/env_model_loader.h"


BaseEnvModel* EnvModelLoader::createEnv(std::string EnvType)
{
    if (EnvType=="occupancy")
    {

       return new OccupancyGridEnv;
    }
    /*
    else if (EnvType=="gridmap")
    {

        BaseEnvModel* MyEnv = new GridMapEnv;
        GridMapEnv *GridEnv;
        GridEnv = dynamic_cast<GridMapEnv*>(MyEnv);


        return GridEnv;
    
    }   
    */                        
}
