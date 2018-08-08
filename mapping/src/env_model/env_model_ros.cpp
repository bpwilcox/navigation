#include "env_model/base_env_model.h"
#include "ros/ros.h"
#include "ros/console.h"


#include "env_model/env_model_ros.h"

#include "env_model_msgs/FloatToInt.h"
#include "env_model_msgs/IntToFloat.h"
#include "env_model_msgs/GetValueAt.h"
#include "env_model_msgs/GetValueAtPos.h"
#include "env_model_msgs/IsMapValid.h"
#include "env_model_msgs/GetRayTrace.h"

        
EnvModelROS::EnvModelROS()
{   
    // User defined variables (can be defined outside and passed through constructor)
    map_type = "occupancy";
    base_layer = "static";
    base_map_srv = "static_occ_grid";

    // Factory Pattern
    m = new EnvModelLoader();
    
    // Create appropriate map object
    MyEnv = m->createEnv(map_type);

    //Initialize Map from Server into layer (default as static)
    MyEnv->initializeMapFromServer(base_layer, base_map_srv);



    getPos_srv = n.advertiseService("getPos", &EnvModelROS::getPosCallback, this);
    getCell_srv = n.advertiseService("getCell", &EnvModelROS::getCellIndexCallback, this);
    getRayTrace_srv = n.advertiseService("rayTrace", &EnvModelROS::getRayTraceCallback, this);
    isValid_srv = n.advertiseService("isValid", &EnvModelROS::isMapValidCallback, this);
    getValueAt_srv = n.advertiseService("getValueAt", &EnvModelROS::getValueAtCallback, this);
    getValueAtPos_srv = n.advertiseService("getValueAtPos", &EnvModelROS::getValueAtPosCallback, this);

    //toMap_srv = n.advertiseService("toMap", &EnvModelROS::getPosCallback, this);
    //toWorld_srv = n.advertiseService("toWorld", &EnvModelROS::getPosCallback, this);    
}

    
bool EnvModelROS::getPosCallback(env_model_msgs::IntToFloat::Request  &req,
                env_model_msgs::IntToFloat::Response &res)
{
    
    double x, y;
    x = MyEnv->getPosX(req.i[0]);
    y = MyEnv->getPosY(req.i[1]);
    std::vector<double> pos = {x, y};
    res.d  = pos;
    
    return true;
}

bool EnvModelROS::getCellIndexCallback(env_model_msgs::FloatToInt::Request  &req,
                env_model_msgs::FloatToInt::Response &res)
{
    
    int i, j;
    i = MyEnv->getCellX(req.d[0]);
    j = MyEnv->getCellY(req.d[1]);
    std::vector<long int> index = {i, j};
    res.i  = index;
    
    return true;
}
bool EnvModelROS::isMapValidCallback(env_model_msgs::IsMapValid::Request  &req,
                env_model_msgs::IsMapValid::Response &res)
{
    res.isvalid = MyEnv->isValid(req.index[0],req.index[1]);
    return true;
}  

bool EnvModelROS::getValueAtCallback(env_model_msgs::GetValueAt::Request  &req,
                env_model_msgs::GetValueAt::Response &res)
{
    res.value = MyEnv->getValueAt(req.layer,req.index[0],req.index[1]);
    return true;
}   


bool EnvModelROS::getValueAtPosCallback(env_model_msgs::GetValueAtPos::Request  &req,
                env_model_msgs::GetValueAtPos::Response &res)
{
    res.value = MyEnv->getValueAtPos(req.layer,req.pos[0],req.pos[1]);
    return true;
}

bool EnvModelROS::getRayTraceCallback(env_model_msgs::GetRayTrace::Request  &req,
                env_model_msgs::GetRayTrace::Response &res)
{
    res.range = MyEnv->getRayTrace(base_layer,req.x, req.y, req.a, req.max);
    return true;
     
}       



int main(int argc, char **argv)
{
    ros::init(argc, argv, "env_model");
    try
    {
        EnvModelROS em;
        ros::spin();
    }
    catch(std::runtime_error& e)
    {
        ROS_ERROR("env_model exception: %s", e.what());
        return -1;
    }

    return 0;
}        
      
