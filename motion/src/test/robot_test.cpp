#include <stdlib.h>
#include <iostream>
#include "base_control.h"
#include "ros/ros.h"
#define _DEBUG
#define FORWARD 0
#define BACK    1
#define RIGHT   2
#define LEFT    3
#define ROTATION 4
bool flag = false;
void inturrupt(int signal)
{
    printf("get SIGINT %d\n", signal);
    flag = true;
}
int main(int argc, char** argv)
{
    int speed = 10;
    int state = FORWARD;
    std::string dir;
    if(argc == 2){
        speed = strtol(argv[1], NULL, 10);
    }
    if(argc == 3){
        speed = strtol(argv[2], NULL, 10);
        dir = argv[1];
        if(dir == "-r")state = RIGHT;
        else if(dir == "-f")state = FORWARD;
        else if(dir == "-b")state = BACK;
        else if(dir == "-l")state = LEFT;
        else if(dir == "-ro")state = ROTATION;
        else{
            printf("ussage [direction(-r/-b/-l/-f)] [speed %%]\n");
            exit(EXIT_FAILURE);
        }
    }
    ros::init(argc, argv, "robot_test");
    ros::NodeHandle n;
    BaseControl Base(argc, argv, true);

    serial_rx RX;
    robot_command CMD = {0};
    robot_command RobotVel = {0};
    robot_command RobotTraj = {0};

    signal(SIGINT, inturrupt);
    ros::Rate loop_rate(30);
    double scale;
    double real_rpm1, real_rpm2, real_rpm3;
    double target_rpm1, target_rpm2, target_rpm3;
    int counter;
    while(true){
        if(flag){
            break;
        }
        counter++;
        switch(state){
            case FORWARD:
                printf("FORWARD\n");
                CMD.x = speed;
                CMD.y = 0;
                CMD.yaw = 0;
                if(counter > 45){
                    counter = 0;
//                    state = RIGHT;
                }
                break;
            case BACK:
                printf("BACK\n");
                CMD.x = -speed;
                CMD.y = 0;
                CMD.yaw = 0;
                if(counter > 45){
                    counter = 0;
//                    state = LEFT;
                }
                break;
            case LEFT:
                printf("LEFT\n");
                CMD.x = 0;
                CMD.y = speed;
                CMD.yaw = 0;
                if(counter > 45){
                    counter = 0;
//                    state = FORWARD;
                }
                break;
            case RIGHT:
                printf("RIGHT\n");
                CMD.x = 0;
                CMD.y = speed;
                CMD.yaw = 0;
                if(counter > 45){
                    counter = 0;
//                    state = BACK;
                }
                break;
            case ROTATION:
                printf("ROTATION\n");
                CMD.x = 0;
                CMD.y = 0;
                CMD.yaw = speed;
                if(counter > 45){
                    counter = 0;
//                    state = BACK;
                }
                break;

            default:
                CMD.x = 0;
                CMD.y = 0;
                CMD.yaw = 0;
                break;

        }
        Base.Send(CMD);
        if(Base.GetBaseFlag()){
            RX = Base.GetOdoMotor();
            RobotVel = Base.GetOdoRobot();
            RobotTraj = Base.GetTraj();

#ifdef _DEBUG
            printf("\n*****robot command******\n");
            printf("target robot speed: %f,%f,%f\n", CMD.x, CMD.y, CMD.yaw);
            printf("\n*****get feedback******\n");
            printf("real robot speed: %f,%f,%f\n", RobotVel.x, RobotVel.y, RobotVel.yaw);
            printf("real robot speed: %f,%f,%f\n", RobotTraj.x, RobotTraj.y, RobotTraj.yaw);
#endif
        }
        loop_rate.sleep();
    }
    Base.McsslFinish();
    std::cout << "Close Attack Motion\n";


    return 0;
}
