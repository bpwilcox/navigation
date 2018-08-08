#ifndef ENV_MODEL_ENV_MODEL_ROS_H
#define ENV_MODEL_ENV_MODEL_ROS_H

#include "env_model/base_env_model.h"
#include "ros/ros.h"
#include "ros/console.h"

#include "env_model_msgs/FloatToInt.h"
#include "env_model_msgs/IntToFloat.h"
#include "env_model_msgs/GetValueAt.h"
#include "env_model_msgs/GetValueAtPos.h"
#include "env_model_msgs/IsMapValid.h"
#include "env_model_msgs/GetRayTrace.h"



#include "env_model/env_model_loader.h"

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

        
        std::string base_layer;
        std::string base_map_srv;
        std::string map_type;
        
        EnvModelLoader *m;
        BaseEnvModel* MyEnv;

        bool getPosCallback(env_model_msgs::IntToFloat::Request  &req,
                        env_model_msgs::IntToFloat::Response &res);

        bool getCellIndexCallback(env_model_msgs::FloatToInt::Request  &req,
                        env_model_msgs::FloatToInt::Response &res);        

        bool isMapValidCallback(env_model_msgs::IsMapValid::Request  &req,
                        env_model_msgs::IsMapValid::Response &res);  

        bool getValueAtCallback(env_model_msgs::GetValueAt::Request  &req,
                        env_model_msgs::GetValueAt::Response &res);   


        bool getValueAtPosCallback(env_model_msgs::GetValueAtPos::Request  &req,
                        env_model_msgs::GetValueAtPos::Response &res);

        bool getRayTraceCallback(env_model_msgs::GetRayTrace::Request  &req,
                        env_model_msgs::GetRayTrace::Response &res);                                                                                                   
};



#endif