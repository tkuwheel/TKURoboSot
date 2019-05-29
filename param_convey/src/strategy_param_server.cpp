#include "ros/ros.h"
#include <ros/package.h> 
#include "param_convey/strategy_param.h"

bool dump(param_convey::strategy_param::Request  &req,
         param_convey::strategy_param::Response &res)
{
  if(req.receive == 1){
    req.receive = 0;
    ROS_INFO("Updating infos...");
    std::string PackagePath = ros::package::getPath("fira_launch"); 
    // std::cout << PackagePath << "\n";
    std::string path = "rosparam dump "+PackagePath+"/default_config/vision_better.yaml";
    const char *path_dump = path.c_str();
    system(path_dump);
    ROS_INFO("Update Success!");
    res.update = 2;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "strategy_param_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("StrategyParam", dump);
  ROS_INFO("Ready to convey the strategy's parameters.\n");
  ros::spin();

  return 0;
}
