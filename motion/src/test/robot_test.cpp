#include <stdlib.h>
#include <iostream>
#include "base_control.h"
#include "ros/ros.h"
#define DEBUG
#define FORWARD 0
#define BACK    1
#define RIGHT   2
#define LEFT    3
bool flag = false;
void inturrupt(int signal)
{
    printf("get SIGINT %d\n", signal);
    flag = true;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_test");
    ros::NodeHandle n;
    BaseControl Base(argc, argv, true);

    serial_rx RX;
    robot_command CMD;
    robot_command RobotVel;

    signal(SIGINT, inturrupt);
    ros::Rate loop_rate(30);
    double scale;
    double real_rpm1, real_rpm2, real_rpm3;
    double target_rpm1, target_rpm2, target_rpm3;
    int state = FORWARD;
    int counter;
    while(true){
        if(flag){
            break;
        }
        counter++;
        switch(state){
            case FORWARD:
                CMD.x_speed = 100;
                CMD.y_speed = 0;
                CMD.yaw_speed = 0;
                if(counter > 45){
                    counter = 0;
                    state = RIGHT;
                }
                break;
            case BACK:
                CMD.x_speed = -100;
                CMD.y_speed = 0;
                CMD.yaw_speed = 0;
                if(counter > 45){
                    counter = 0;
                    state = LEFT;
                }
                break;
            case LEFT:
                CMD.x_speed = 0;
                CMD.y_speed = 100;
                CMD.yaw_speed = 0;
                if(counter > 45){
                    counter = 0;
                    state = FORWARD;
                }
                break;
            case RIGHT:
                CMD.x_speed = 0;
                CMD.y_speed = -100;
                CMD.yaw_speed = 0;
                if(counter > 45){
                    counter = 0;
                    state = BACK;
                }
                break;
            default:
                CMD.x_speed = 0;
                CMD.y_speed = 0;
                CMD.yaw_speed = 0;
                break;

        }
        if(Base.GetBaseFlag()){
            RX = Base.GetOdoMotor();
            RobotVel = Base.GetOdoRobot();

#ifdef DEBUG_
            printf("\n*****robot command******\n");
            printf("target robot speed: %f,%f,%f\n", CMD.x_speed, CMD.y_speed, CMD.yaw_speed);
            printf("\n*****get feedback******\n");
            printf("real robot speed: %f,%f,%f\n", RobotVel.x_speed, RobotVel.y_speed, RobotVel.yaw_speed);
#endif
        }
        loop_rate.sleep();
    }
    Base.McsslFinish();
    std::cout << "Close Attack Motion\n";


    return 0;
}
