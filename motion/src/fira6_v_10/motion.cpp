/********************************
 *	Include system
 ********************************/
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>
/********************************
 *	Include header files
 ********************************/
#include "motor_data.h"
#include "motion_nodeHandle.h"
#include "base_control.h"
/********************************
 *	Define	
 ********************************/
//typedef void * (*THREADFUNCPTR)(void *);
#define DEBUG
bool flag = 0;
void inturrupt(int signal)
{
    printf("get SIGINT %d\n", signal);
    flag = 1;
}
int main(int argc, char **argv)
{
	Motion_nodeHandle Node(argc, argv);
//    ros::init(argc, argv, "Test");
//    ros::NodeHandle n;
	BaseController Base(argc, argv, false);

	RobotCommand robotCMD;
    RobotCommand robotOdo;
    MotorSpeed currRPM;
    signal(SIGINT, inturrupt);
	std::cout << "ATTACK MOTION IS RUNNING!\n";
	ros::Rate loop_rate(CMD_FREQUENCY);
	while(true){
        if(flag){
            Base.Close();
            Base.ShowCsslCallback();
            currRPM = Base.GetCurrRPM();
//            printf("close\n");
            if((currRPM.w1==0)&&(currRPM.w2==0)&&currRPM.w3==0)
                break;
            loop_rate.sleep();
            continue;
        }
//        Base.SetEnable();
        if(Node.getMotionFlag()){
            robotCMD = Node.getMotion();
#ifdef DEBUG
            printf("\n*****get motion******\n");
            Node.ShowCommand();
#endif
            Base.Send(robotCMD);
        }
        if(Base.GetBaseFlag()){
            currRPM = Base.GetCurrRPM();
            Node.pub_robotFB(Base.GetOdometry());
#ifdef DEBUG
            printf("\n*****get feedback******\n");
            printf("motor1 rpm %f\nmotor2 rpm %f\nmotor3 rpm %f\n", currRPM.w1, currRPM.w2, currRPM.w3);
#endif
        }
		loop_rate.sleep();
	}
	std::cout << "Close Attack Motion\n";
	return 0;
}


