#include "ros/ros.h"
#include "param_convey/strategy_param.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "strategy_param_client");
  if (argc != 2)
  {
    ROS_INFO("usage: strategy_param_client X");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<param_convey::strategy_param>("StrategyParam");
  param_convey::strategy_param srv;
  srv.request.receive = atoll(argv[1]);

  if (client.call(srv))
  {
    ROS_INFO("UPDATED!!!!!!!!!!!!!!!!!!!!");
  }
  else
  {
    ROS_ERROR("Failed to call service strategy_param");
    return 1;
  }

  return 0;
}
