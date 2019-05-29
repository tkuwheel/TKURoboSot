/*
 * csll is for serial port (RS232)
 */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "MotorControl.h"
#include <boost/program_options.hpp>
#include <stdio.h>
#include <termios.h>  // for tcxxxattr, ECHO, etc ..
#include <unistd.h>    // for STDIN_FILENO
#include "std_msgs/Int32.h"

//#define Debug
//#include <conio.h>

using std::string;

//-------variable-------//
const char *motion_topic_name = "/cmd_vel";

//const double mAngle1Cos(cos(5*M_PI/3));
//const double mAngle2Cos(cos(M_PI/3));
//const double mAngle3Cos(cos(M_PI));

//const double mAngle1Sin(sin(5*M_PI/3));
//const double mAngle2Sin(sin(M_PI/3));
//const double mAngle3Sin(sin(M_PI));
const double mAngle1Cos(cos(5*M_PI/6));
const double mAngle2Cos(cos(M_PI/6));
const double mAngle3Cos(cos(3*M_PI/2));

const double mAngle1Sin(sin(M_PI/6));
const double mAngle2Sin(sin(5*M_PI/6));
const double mAngle3Sin(sin(3*M_PI/2));
const double rad2rpm =  60.0  / (2.0*M_PI);
const double percentRange =  PWM_Limit_Percent_Max - PWM_Limit_Percent_Min;

double w1,w2,w3;
double en1,en2,en3;


#define ESCAPE 27


/*==============================================================================*/
//Initialize
/*==============================================================================*/
void Initialize()
{
    w1=0;
    w2=0;
    w3=0;
}
/*==============================================================================*/
//Topic call back
/*==============================================================================*/
void motionCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    w1 = mAngle1Cos*msg->linear.y + mAngle1Sin*msg->linear.x + ROBOT_RADIUS*msg->angular.z*(-1);
    w2 = mAngle2Cos*msg->linear.y + mAngle2Sin*msg->linear.x + ROBOT_RADIUS*msg->angular.z*(-1);
    w3 = mAngle3Cos*msg->linear.y + mAngle3Sin*msg->linear.x + ROBOT_RADIUS*msg->angular.z*(-1);

    en1 = 1; en2 = 1; en3 = 1;
//    printf("=====================================================\n");
    if(w1 != 0 || w2!=0 || w3!=0){
//	printf("-->w1:%0.2f w2:%0.2f w3:%0.2f en1:%f en2:%f en3:%f<--\n\n",w1,w2,w3,en1,en2,en3);
    }

//    ROS_INFO("<=============Kinematics=================>");
//    ROS_INFO("w1:%f\tw2:%f\tw3:%f\n",w1,w2,w3);
//    ROS_INFO("<========================================>");
    //speed2pwm(en);
    //ROS_INFO("w1:%f\tw2:%f\tw3:%f\n",w1,w2,w3);
    //ROS_INFO("*******************************");

    mcssl_send2motor(w1,w2,w3,en1,en2,en3,0);
}

void shootCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int shoot_power = msg->data;
    char shoot=(char)shoot_power;
    mcssl_send2motor(w1,w2,w3,en1,en2,en3,shoot);
}

/*==============================================================================*/
//Main
/*==============================================================================*/
int main(int argc, char **argv)
{
    //Initial
    ros::init(argc, argv, "motion");
    ros::NodeHandle n("~");

//    std::string port_name;
//    n.param<std::string>("port", port_name, "/dev/ttyUSB3");

//    ROS_INFO("port3=%s",port_name.c_str());



    //if(mcssl_init()<=0){return 0;}
//    ROS_INFO("Initialize Motion with port=%s...",port_name.c_str());
    char *command;
    //scanf("%s", command);

    Initialize();

    geometry_msgs::Twist feedback_msg;
    //motion subscriber
    ros::Subscriber motion_sub = n.subscribe(motion_topic_name, 1, motionCallback);
    ros::Subscriber shoot_sub  = n.subscribe("/shoot",1,shootCallback);
    ros::Publisher feedback_pub  = n.advertise<geometry_msgs::Twist>("/motorFB",0);
#ifdef Debug
    ROS_INFO("Debug mode (motion)\n");
#else
    do{

        if(mcssl_init(/*port_name.c_str()*/) > 0){
            break;
        }else{
          usleep(1000000);//1s = 1,000,000 us
        }

    }while(ros::ok());
#endif
    ROS_INFO("Motion is running\n");
    ros::Rate loop_rate(30);
//    static double time = 0;
    while(ros::ok())
    {
//	mcssl_send2motor(w1,w2,w3);
#ifdef Debug

#else
        feedback_msg = getmotionfeedback();
        feedback_pub.publish(feedback_msg);
#endif

//        double n_time,duration;
//        n_time = ros::Time::now().toSec();
//        duration = n_time - time;
//        time = n_time;
//        printf("\nduration = %f ",duration);
        //spin
        ros::spinOnce();
        loop_rate.sleep();
    }
    //RS232 finish
    mcssl_finish();

    return 0;
}
