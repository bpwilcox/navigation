#include "ros/ros.h"
#include "ros/console.h"
#include "env_model_msgs/IntToFloat.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "env_model_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<env_model_msgs::IntToFloat>("getPos");
  env_model_msgs::IntToFloat srv;
  srv.request.i = {2,3};
  if (client.call(srv))
  {
    ROS_INFO("d: %f, %f", srv.response.d[0], srv.response.d[1]);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
