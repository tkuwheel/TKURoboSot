#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "nubot_common/OminiVisionInfo.h"
#include "transfer/PPoint.h"

using namespace message_filters;

void callback(const nubot_common::OminiVisionInfoConstPtr &omni, const transfer::PPointConstPtr &goal)
{
  std::cout << "Ball Info: " << omni->ballinfo.real_pos.radius << ", "
            <<  omni->ballinfo.real_pos.angle << "\n"
            << "Robot Info: " << omni->robotinfo[0].pos.x << ", "
            << omni->robotinfo[0].pos.y << "\n"
            << "Goal Info: " << goal->right_radius << ", "
            << goal->right_angle << ", "
            << goal->left_radius << ", "
            << goal->left_angle << std::endl;
}

int main(int argc, char **argv)
{
  std::stringstream ss1, ss2;
  ss1 << "/nubot" << argv[1] << "/omnivision/OmniVisionInfo";
  ss2 << "/nubot" << argv[1] << "/omnivision/OmniVisionInfo/GoalInfo";
  std::cout<<"Sub: \n"<<ss1.str()<<", "<<ss2.str()<<std::endl;
  ros::init(argc, argv, "gazebo_relay_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<nubot_common::OminiVisionInfo> omni_sub(nh, ss1.str(), 1);
  message_filters::Subscriber<transfer::PPoint> goal_sub(nh, ss2.str(), 1);

  // typedef sync_policies::ExactTime<nubot_common::OminiVisionInfo, transfer::PPoint> MySyncPolicy;
  // typedef message_filters::sync_policies::ApproximateTime<nubot_common::OminiVisionInfo, transfer::PPoint> MySyncPolicy;

  // Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), omni_sub, goal_sub);
  TimeSynchronizer<nubot_common::OminiVisionInfo, transfer::PPoint> sync(omni_sub, goal_sub, 100);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}