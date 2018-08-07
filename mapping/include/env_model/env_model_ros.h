#ifndef ENV_MODEL_ENV_MODEL_ROS_H
#define ENV_MODEL_ENV_MODEL_ROS_H

#include "env_model/base_env_model.h"
#include "ros/ros.h"
#include "ros/console.h"
#include <std_msgs/Float64.h>
#include "env_model_msgs/IntToFloat.h"

class EnvModelROS 
{


    public:
        
        
        EnvModelROS();

        ~EnvModelROS(){};
    private:
        

        ros::NodeHandle n;   
        ros::Publisher mypub;
        ros::ServiceServer getPos_srv;
        ros::ServiceServer getCell_srv;
        ros::ServiceServer toMap_srv;
        ros::ServiceServer toWorld_srv;
        ros::ServiceServer getRayTrace_srv;
        ros::ServiceServer isValid_srv;
        ros::ServiceServer getValueAt_srv;
        ros::ServiceServer getValueAtPos_srv;


        bool getPosCallback(env_model_msgs::IntToFloat::Request  &req,
                        env_model_msgs::IntToFloat::Response &res);

        
};



#endif