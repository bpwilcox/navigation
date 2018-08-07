#include "env_model/base_env_model.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "env_model_msgs/IntToFloat.h"
#include "env_model/env_model_ros.h"

        
EnvModelROS::EnvModelROS()
{
    getPos_srv = n.advertiseService("getPos", &EnvModelROS::getPosCallback, this);
    getCell_srv = n.advertiseService("getCell", &EnvModelROS::getPosCallback, this);
    toMap_srv = n.advertiseService("toMap", &EnvModelROS::getPosCallback, this);
    toWorld_srv = n.advertiseService("toWorld", &EnvModelROS::getPosCallback, this);
    getRayTrace_srv = n.advertiseService("rayTrace", &EnvModelROS::getPosCallback, this);
    isValid_srv = n.advertiseService("isValid", &EnvModelROS::getPosCallback, this);
    getValueAt_srv = n.advertiseService("getValueAt", &EnvModelROS::getPosCallback, this);
    getValueAtPos_srv = n.advertiseService("getValueAtPos", &EnvModelROS::getPosCallback, this);
    
    //mypub = n.advertise<std_msgs::Float64>("myvalue", 1, true);
    //mypub.publish(1.0);
}

    
bool EnvModelROS::getPosCallback(env_model_msgs::IntToFloat::Request  &req,
                env_model_msgs::IntToFloat::Response &res)
{
    
    double x, y;
    x = req.i[0]*1.0;
    y = req.i[1]*2.0;
    std::vector<double> pos = {x, y};
    res.d  = pos;

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
      
