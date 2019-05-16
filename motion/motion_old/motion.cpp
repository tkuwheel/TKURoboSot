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
char en1,en2,en3;


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
//    if(w1 != 0 || w2!=0 || w3!=0){
//	printf("-->w1:%0.2f w2:%0.2f w3:%0.2f en1:%f en2:%f en3:%f<--\n\n",w1,w2,w3,en1,en2,en3);
//    }

//    ROS_INFO("<=============Kinematics=================>");
//    ROS_INFO("w1:%f\tw2:%f\tw3:%f\n",w1,w2,w3);
//    ROS_INFO("<========================================>");
    //speed2pwm(en);
    //ROS_INFO("w1:%f\tw2:%f\tw3:%f\n",w1,w2,w3);
    //ROS_INFO("*******************************");

    mcssl_send2motor(w1,w2,w3,en1,en2,en3,0);
}

void test(){
 unsigned int a,b,c,d;
 int want;

 a = 3; b = 5; c = 1 ; d=1;
 //want = a  + b  << 8 + c << 16 + d << 24;
 want = (c << 8 )+ a;
 printf("want=%d\n",want);


}
void shootCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int shoot_time;
    int count_speed_time;
    shoot_time = msg->data;
    if(shoot_time>250)shoot_time=250;
//    printf("tell me\n");
    for(count_speed_time=0;count_speed_time<=shoot_time;count_speed_time++){
        mcssl_send2motor(w1,w2,w3,en1,en2,en3,1);
    }
    for(count_speed_time=0;count_speed_time<=shoot_time;count_speed_time++){
        mcssl_send2motor(w1,w2,w3,en1,en2,en3,0);
    }
//    mcssl_send2motor(w1,w2,w3,en1,en2,en3,0);
//    printf("shoot_time:%d\n",msg->data);
}
int getch (void)
{
    int ch;
    struct termios oldt, newt;

    tcgetattr(STDIN_FILENO, &oldt);
    memcpy(&newt, &oldt, sizeof(newt));
    newt.c_lflag &= ~( ECHO | ICANON | ECHOE | ECHOK |
                       ECHONL | ECHOPRT | ECHOKE | ICRNL);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
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


    int c ;



    Initialize();

    geometry_msgs::Twist feedback_msg;
    //motion subscriber
    ros::Subscriber motion_sub = n.subscribe(motion_topic_name, 1000, motionCallback);
//    ros::Subscriber shoot_sub  = n.subscribe("/shoot",1,shootCallback);
    ros::Publisher motor_pub  = n.advertise<geometry_msgs::Twist>("/motorFB",1000);
    ros::Publisher location_pub  = n.advertise<geometry_msgs::Twist>("/location",1000);
    do{

        if(mcssl_init(/*port_name.c_str()*/) > 0){
            break;
        }else{
          usleep(1000000);//1s = 1,000,000 us
        }

    }while(ros::ok());
    ROS_INFO("Motion is running\n");
    ros::Rate loop_rate(50);

    while(ros::ok())
    {
	//mcssl_send2motor(w1,w2,w3);
        feedback_msg = getmotorfeedback();
        motor_pub.publish(feedback_msg);
        feedback_msg = getlocation();
        location_pub.publish(feedback_msg);
        //spin
        ros::spinOnce();
        loop_rate.sleep();
    }
    //RS232 finish
    mcssl_finish();

    return 0;
}
