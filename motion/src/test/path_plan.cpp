/*
 *
 */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <unistd.h>

using std::string;

//-------variable-------//

double FB_x,FB_y,FB_yaw;
double motorFB_x,motorFB_y,motorFB_yaw;

#define ESCAPE 27

void motorFBCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    motorFB_x = msg->linear.x;
    motorFB_y = msg->linear.y;
    motorFB_yaw = msg->angular.z;
}

void locationCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    FB_x = msg->linear.x;
    FB_y = msg->linear.y;
    FB_yaw = msg->angular.z;
}

/*==============================================================================*/
//Main
/*==============================================================================*/
int main(int argc, char **argv)
{

    //Initial
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n("~");



    geometry_msgs::Twist cmd_msg;
    //motion subscriber
    ros::Subscriber motorFB_sub = n.subscribe("/motorFB", 1000, motorFBCallback);
    ros::Subscriber location_sub = n.subscribe("/location", 1000, locationCallback);
    ros::Publisher motion_pub  = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    double x,y,yaw;
    if(argc != 4){
        ROS_ERROR("Enter the wrong arguments\n");
        ROS_INFO("usage: add three double value, x y yaw");
        return 1;
    }else{
        x = atof(argv[1]);
        y = atof(argv[2]);
        yaw = atof(argv[3]);
    }
    ros::Rate loop_rate(10);
    while(ros::ok()){
        if((FB_x||FB_y||FB_yaw)!=0){
            break;
        }
        ros::spinOnce();
    }
    double v_x,v_y,v_yaw;
    while(ros::ok())
    {
        printf("location (x,y,yaw) = (%f,%f,%f)\n",FB_x,FB_y,FB_yaw*180/M_PI);
        printf("distance (x,y,yaw) = (%f,%f,%f)\n",x-FB_x,y-FB_y,(yaw-FB_yaw)*180/M_PI);
        if(fabs(x-FB_x)>0.01){
//            v_x = ((x-motorFB_x)>0)?5:-5;
            v_x = ((x-FB_x)>0)?5*cos(motorFB_yaw):-5*cos(motorFB_yaw);
        }else{
            v_x = 0;
        }
        if(fabs(y-FB_y)>0.01){
//            v_y = ((y-motorFB_y)>0)?5:-5;
            v_y = ((y-FB_y)>0)?5*cos(motorFB_yaw):-5*cos(motorFB_yaw);
        }else{
            v_y = 0;
        }
        if(fabs(yaw-FB_yaw)*180/M_PI>0.3){
            v_yaw = ((yaw-FB_yaw)*180/M_PI>0.3)?5:((yaw-FB_yaw)*180/M_PI<-0.3)?-5:0;
        }else{
            v_yaw = 0;
        }
        cmd_msg.linear.x = v_x;
        cmd_msg.linear.y = v_y;
        cmd_msg.angular.z = v_yaw;
        motion_pub.publish(cmd_msg);
        printf("velocity (x,y,yaw) = (%f,%f,%f)\n\n",cmd_msg.linear.x,cmd_msg.linear.y,cmd_msg.angular.z);
        if((v_x||v_y||v_yaw)!=0);
        else break;
        ros::spinOnce();
        loop_rate.sleep();


    }
    //RS232 finish
    motion_pub.publish(cmd_msg);
    return 0;
}
