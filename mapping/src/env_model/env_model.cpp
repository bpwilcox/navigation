#include "ros/ros.h"
#include "ros/console.h"
#include "env_model/env_models.h"
#include "env_model/env_model_loader.h"


/*

Goal is to:
1) Grab map from server 
2) Initialize appropriate object for map
3) Make map available for service calls

*/


nav_msgs::OccupancyGrid map;


int main(int argc, char **argv)
{
    // Factory
    EnvModelLoader *m = new EnvModelLoader();
    BaseEnvModel* MyEnv = m->createEnv("occupancy");
    //MyEnv->getMap("static_occ_grid");
    ROS_INFO("value %f", MyEnv->getPosX(1));

    //OccupancyGridEnv *OccEnv;
    //OccEnv = dynamic_cast<OccupancyGridEnv*>(MyEnv);

    



}
