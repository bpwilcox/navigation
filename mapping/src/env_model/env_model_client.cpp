#include "ros/ros.h"
#include "ros/console.h"
#include "env_model_msgs/IntToFloat.h"
#include "env_model_msgs/FloatToInt.h"
#include "env_model_msgs/GetValueAt.h"
#include "env_model_msgs/GetValueAtPos.h"
#include "env_model_msgs/IsMapValid.h"
#include "env_model_msgs/GetRayTrace.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "env_model_client");

  ros::NodeHandle n;

  // Test all Environmental Model services
  ros::ServiceClient Pos_client = n.serviceClient<env_model_msgs::IntToFloat>("getPos");
  ros::ServiceClient Cell_client = n.serviceClient<env_model_msgs::FloatToInt>("getCell");
  ros::ServiceClient ValueAt_client = n.serviceClient<env_model_msgs::GetValueAt>("getValueAt");
  ros::ServiceClient ValueAtPos_client = n.serviceClient<env_model_msgs::GetValueAtPos>("getValueAtPos");
  ros::ServiceClient IsValid_client = n.serviceClient<env_model_msgs::IsMapValid>("isValid");
  ros::ServiceClient RayTrace_client = n.serviceClient<env_model_msgs::GetRayTrace>("rayTrace"); 
  
  env_model_msgs::IntToFloat Pos_srv;
  env_model_msgs::FloatToInt Cell_srv;
  env_model_msgs::GetValueAt ValueAt_srv;
  env_model_msgs::GetValueAtPos ValueAtPos_srv;
  env_model_msgs::IsMapValid IsValid_srv;
  env_model_msgs::GetRayTrace RayTrace_srv;

  Pos_srv.request.i = {2,3};
  if (Pos_client.call(Pos_srv))
  {
    ROS_INFO("position [m]: %f, %f", Pos_srv.response.d[0], Pos_srv.response.d[1]);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  Cell_srv.request.d = {5,5};
  if (Cell_client.call(Cell_srv))
  {
    ROS_INFO("index: %ld, %ld", Cell_srv.response.i[0], Cell_srv.response.i[1]);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  ValueAt_srv.request.layer = "distance";
  ValueAt_srv.request.index = {5,5};
  if (ValueAt_client.call(ValueAt_srv))
  {
    ROS_INFO("value: %f", ValueAt_srv.response.value);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  ValueAtPos_srv.request.layer = "static";
  ValueAtPos_srv.request.pos = {5,5};
  if (ValueAtPos_client.call(ValueAtPos_srv))
  {
    ROS_INFO("value: %f", ValueAtPos_srv.response.value);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  IsValid_srv.request.index = {5,5};
  if (IsValid_client.call(IsValid_srv))
  {
    ROS_INFO("valid: %d", IsValid_srv.response.isvalid);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  RayTrace_srv.request.x = 5;
  RayTrace_srv.request.y = 5;
  RayTrace_srv.request.a = 0;
  RayTrace_srv.request.max = 100;

  if (RayTrace_client.call(RayTrace_srv))
  {
    ROS_INFO("range: %f", RayTrace_srv.response.range);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }
  
  return 0;
}
