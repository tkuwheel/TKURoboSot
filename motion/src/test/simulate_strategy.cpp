#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
int main(int argc, char** argv)
{
	ros::init(argc, argv, "simulate_strategy");
	ros::NodeHandle n;
	ros::Publisher pub_motion = n.advertise<geometry_msgs::Twist>("/motion/cmd_vel",1000);
	ros::Publisher pub_shoot = n.advertise<std_msgs::Int32>("/motion/shoot", 1000);
	double x_speed = 0;
	double y_speed = 0;
	double yaw_speed = 0;
	geometry_msgs::Twist motion_msg;
	std_msgs::Int32 shoot_msg;
	shoot_msg.data=0;
	ros::Rate loop_rate(50);
	int counter = 0;
	while(ros::ok()){
		shoot_msg.data = (shoot_msg.data%100);
		counter = counter%720;
		if(counter<240){
			y_speed = 0;
			yaw_speed = 0;
			if(counter>120){
				x_speed+=0.2;
				shoot_msg.data++;
			}else if(counter == 120){
				x_speed = 0;
				shoot_msg.data = 0;
			}else{
				x_speed-=0.2;
			}
		}else if(counter<480){
			x_speed = 0;
			yaw_speed = 0;
			if(counter>360){
				y_speed+=0.2;
				shoot_msg.data++;
			}else if(counter == 360){
				y_speed = 0;
				shoot_msg.data = 0;
			}else{
				y_speed-=0.2;
			}

		}else{
			x_speed = 0;
			y_speed = 0;
			if(counter>600){
				yaw_speed+=0.2;
			}else if(counter == 600){
				yaw_speed = 0;
			}else{
				yaw_speed-=0.2;
				shoot_msg.data++;
			}
		}
		motion_msg.linear.x = x_speed;
		motion_msg.linear.y = y_speed;
		motion_msg.angular.z = yaw_speed;
		pub_motion.publish(motion_msg);
		pub_shoot.publish(shoot_msg);
		counter++;
		ros::spinOnce();
		loop_rate.sleep();
	}
}
