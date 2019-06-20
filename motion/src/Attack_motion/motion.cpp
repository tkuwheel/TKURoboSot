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
#define DEBUG_M_FB
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
	BaseControl Base;

	robot_command robotCMD;
    serial_rx rx;
    serial_rx* RX = &rx;
//	robot_command main_robotFB;
//	int count = 0;
	//while(ros::ok()){
	//	if(Global_Motor_Control.mcssl_init()){
	//		break;
	//	}else{
	//		exit(EXIT_FAILURE);
	//	}
	//}
    signal(SIGINT, inturrupt);
	std::cout << "ATTACK MOTION IS RUNNING!\n";
	ros::Rate loop_rate(1);
	while(true){
        if(flag){
            break;
        }
        if(Node.getMotionFlag()){
            robotCMD = Node.getMotion();
#ifdef DEBUG
            printf("\n*****get motion******\n");
            std::cout << "x: " << robotCMD.x_speed << "\t";
            std::cout << "y: " << robotCMD.y_speed << "\t";
            std::cout << "yaw: " << robotCMD.yaw_speed << "\t";
            std::cout << "\nshoot power: " << robotCMD.shoot_power << "\t";
            std::cout << "hold: " << robotCMD.hold_ball << "\t";
            std::cout << "remote: " << robotCMD.remote;
            std::cout << std::endl;
#endif
            Base.Send(robotCMD);
        }
        if(Base.GetBaseFlag()){
            rx = Base.GetOdo();
#ifdef DEBUG
            printf("\n*****get feedback******\n");
            std::cout << std::dec;
            std::cout << "id: " << RX->id << "\t";
            std::cout << "size: " << RX->size << "\t";
            std::cout << "duration: " << RX->duration << "\t\n";
            std::cout << "w1: " << RX->w1 * 600  / 2000 << "\t\n";
            std::cout << "w2: " << RX->w2 << "\t\n";
            std::cout << "w3: " << RX->w3 << "\t\n";
#endif
        }
		loop_rate.sleep();
	}
    Base.McsslFinish();
	std::cout << "Close Attack Motion\n";
	return 0;
}


