#include "motion_nodeHandle.h"
Motion_nodeHandle::Motion_nodeHandle(int argc, char **argv)
{
	this->node_robotCMD = new robot_command;
	this->node_robotCMD->x_speed = new double;
	this->node_robotCMD->y_speed = new double;
	this->node_robotCMD->yaw_speed = new double;
	this->node_robotCMD->shoot_power = new int;
	this->node_RX = new serial_rx;
	std::memset(this->node_robotCMD->x_speed, 0, sizeof(double));
	std::memset(this->node_robotCMD->y_speed, 0, sizeof(double));
	std::memset(this->node_robotCMD->yaw_speed, 0, sizeof(double));
	std::memset(this->node_robotCMD->shoot_power, 0, sizeof(int));
	this->remote = false;
#ifdef DEBUG
	std::cout << "Motion_nodeHandle(DEBUG)\n";
	std::cout << "x_speed: " <<*this->node_robotCMD->x_speed << std::endl;
	std::cout << "y_speed: " << *this->node_robotCMD->y_speed << std::endl;
	std::cout << "yaw_speed: " << *this->node_robotCMD->yaw_speed << std::endl;
	std::cout << "shoot_power: " << *this->node_robotCMD-> shoot_power << std::endl;
#endif
	init(argc, argv);
}

Motion_nodeHandle::~Motion_nodeHandle()
{
	ros::shutdown();
#ifdef DEBUG
	std::cout << "~Motion_nodeHandle(DEBUG)\n";
#endif
}

void Motion_nodeHandle::init(int argc, char **argv)
{
	ros::init(argc, argv, "Attack_motion");
#ifdef DEBUG
	std::cout << "nodeHandle init(DEBUG)\n";
	std::cout << "PATH= " << *argv << std::endl;
#endif
	n = new ros::NodeHandle();
	motionFB_pub = n->advertise<geometry_msgs::Twist>(motion_feedback_topic_name,1000);
	motion_sub = n->subscribe<geometry_msgs::Twist>(motion_topic_name, 1000, &Motion_nodeHandle::motionCallback, this);
	shoot_sub = n->subscribe<std_msgs::Int32>(shoot_topic_name, 1000, &Motion_nodeHandle::shootCallback, this);
	remote_sub = n->subscribe<std_msgs::Bool>(remote_topic_name, 1000, &Motion_nodeHandle::remoteCallback, this);
}

void Motion_nodeHandle::motionCallback(const geometry_msgs::Twist::ConstPtr &motion_msg)
{
	//this->x_speed = motion_msg->linear.x;
	//this->y_speed = motion_msg->linear.y;
	//this->yaw_speed = motion_msg->angular.z;
	*(this->node_robotCMD->x_speed) = motion_msg->linear.x;
	*(this->node_robotCMD->y_speed) = motion_msg->linear.y;
	*(this->node_robotCMD->yaw_speed) = motion_msg->angular.z;
#ifdef DEBUG
	std::cout << "motionCallback(DEBUG)\n";
	std::cout << std::dec;
	std::cout << "X axis speed(%): " << *(this->node_robotCMD->x_speed) << std::endl;
	std::cout << "Y axis speed(%): " << *(this->node_robotCMD->y_speed) << std::endl;
	std::cout << "yaw speed(%): " << *(this->node_robotCMD->yaw_speed) << std::endl;
	std::cout << std::endl;
#endif
}


void Motion_nodeHandle::shootCallback(const std_msgs::Int32::ConstPtr &shoot_msg)
{
	*(this->node_robotCMD->shoot_power) = shoot_msg->data;
#ifdef DEBUG
	std::cout << "shootCallback(DEBUG)\n";
	std::cout << std::dec;
	std::cout << "shoot power(%): " << *(node_robotCMD->shoot_power) << std::endl;
	std::cout << std::endl;
#endif
}

void Motion_nodeHandle::remoteCallback(const std_msgs::Bool::ConstPtr &remote_msg)
{
	this->remote = remote_msg->data;
#ifdef DEBUG
	std::cout << "remoteCallback(DEBUG)\n";
	//std::cout << std::dec;
	std::cout << "remote : " << this->remote << std::endl;
	std::cout << std::endl;

#endif
}

void Motion_nodeHandle::pub(const geometry_msgs::Twist &pub_msgs)
{
	motionFB_pub.publish(pub_msgs);
}

robot_command* Motion_nodeHandle::getMotion()
{
	return this->node_robotCMD;
}

void Motion_nodeHandle::pub_robotFB(robot_command* node_robotFB)
{
	geometry_msgs::Twist robotFB;
	robotFB.linear.x = *(node_robotFB->x_speed);
	robotFB.linear.y = *(node_robotFB->y_speed);
	robotFB.angular.z = *(node_robotFB->yaw_speed);
	pub(robotFB);
}

void Motion_nodeHandle::clear()
{
	if(this->remote == false){
		*(this->node_robotCMD->x_speed) = 0;
		*(this->node_robotCMD->y_speed) = 0;
		*(this->node_robotCMD->yaw_speed) = 0;
	}
	*(this->node_robotCMD->shoot_power) = 0;
}
