#ifndef motion_nodeHandle_H
#define motion_nodeHandle_H
/*********************
 ** Include system
 *********************/
#include <iostream>
#include <cstring>
/*********************
 ** Include ROS
 *********************/
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

/*********************
 ** Include library->
 *********************/
#include "motor_data.h"
/*********************
 ** Define 
 *********************/
#define odometry_topic_name "/motion/odom"
#define motion_feedback_topic_name "/motion/motionFB"
#define motion_topic_name "/motion/cmd_vel"
#define shoot_topic_name "/motion/shoot"
#define remote_topic_name "/motion/remote"

//#define DEBUG 
class Motion_nodeHandle{
public:
	Motion_nodeHandle(int argc, char **argv);
	virtual ~Motion_nodeHandle();
	
private:
	ros::NodeHandle *n;
	ros::Publisher motionFB_pub;
	ros::Subscriber motion_sub;
	ros::Subscriber shoot_sub;
	ros::Subscriber remote_sub;
	robot_command *node_robotCMD;
	serial_rx* node_RX;
	bool remote;
private:
	void init(int argc, char **argv);
	void motionCallback(const geometry_msgs::Twist::ConstPtr &);
	void shootCallback(const std_msgs::Int32::ConstPtr &);
	void remoteCallback(const std_msgs::Bool::ConstPtr &);
	void pub(const geometry_msgs::Twist &);
public:
	robot_command* getMotion();
	void pub_robotFB(robot_command*);
	void clear();
};
#endif
