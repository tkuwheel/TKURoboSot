#ifndef NodeHandle_H
#define NodeHandle_H
/*********************
 ** Include system
 *********************/
#include <iostream>
#include <cstring>
#include <pthread.h>
//#include <signal.h>
/*********************
 ** Include ROS
 *********************/
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

/*********************
 ** Include header files
 *********************/
#include "motor_data.h"
/*********************
 ** Define 
 *********************/
#define odometry_topic_name "motion/odom"
#define motion_feedback_topic_name "motion/motionFB"
#define motion_topic_name "motion/cmd_vel"
#define shoot_topic_name "motion/shoot"
#define remote_topic_name "motion/remote"
#define holdBall_topic_name "motion/hold_ball"

typedef void * (*THREADFUNCPTR)(void *);
//#define DEBUG 
class Motion_nodeHandle{
public:
	Motion_nodeHandle(int, char **);
	virtual ~Motion_nodeHandle();
	
private:
	ros::NodeHandle *n;
	ros::Publisher motionFB_pub;
	ros::Subscriber motion_sub;
	ros::Subscriber shoot_sub;
	ros::Subscriber remote_sub;
    ros::Subscriber holdBall_sub;
	RobotCommand robotCMD;
    pthread_t tid;
	bool remote;
    bool holdBall;
    bool motion_flag;
private:
    static void* mpThreadRun(void* p);
	void init(int argc, char **argv);
	void motionCallback(const geometry_msgs::Twist::ConstPtr &);
	void shootCallback(const std_msgs::Int32::ConstPtr &);
	void remoteCallback(const std_msgs::Bool::ConstPtr &);
    void holdBallCallback(const std_msgs::Bool::ConstPtr &);
	void pub(const geometry_msgs::Twist &);
    void mRun();
public:
//    void *run();
	RobotCommand getMotion();
	void    pub_robotFB(RobotCommand);
	void    clearShoot();
	int     clearAll();
	bool    getMotionFlag();
    void    ShowCommand();
};
#endif //NodeHandle_H
