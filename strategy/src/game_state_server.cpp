#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <strategy/GameStateConfig.h>

void callback(strategy::GameStateConfig &config, uint32_t level) {
  ROS_INFO("Game Start: %s,\tTeam Selected: %s,\tGame State: %s",
            config.game_start?"True":"False",
            config.side.c_str(),
            config.game_state.c_str());
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "game_state_server");

  dynamic_reconfigure::Server<strategy::GameStateConfig> server;
  dynamic_reconfigure::Server<strategy::GameStateConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}