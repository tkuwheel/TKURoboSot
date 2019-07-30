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
#include "parameter.h"
#include "motor_data.h"
#include "node_handle.h"
#include "base_control.h"
/********************************
 *	Define	
 ********************************/
//typedef void * (*THREADFUNCPTR)(void *);
#define DEBUG_
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
    BaseController Base(
            argc, 
            argv, 
            HEAD_1,
            HEAD_2,
            MAX_MOTOR_RPM,
            MIN_MOTOR_RPM,
            MAX_PWM,
            MIN_PWM,
            TICKS_PER_ROUND,
            false
            );
    RobotCommand robotCMD = {0};
    RobotCommand robotOdo = {0};
    MotorSpeed currRPM;
    signal(SIGINT, inturrupt);

    int count_cmd = 0; // command
    int count_fb = 0; // feedback
    ros::Rate loop_rate(CMD_FREQUENCY);
    std::cout << "FIRA6th MOTION IS RUNNING!\n";
    while(true){
        if(flag){
            Base.Close();
            Base.ShowCsslCallback();
            currRPM = Base.GetCurrRPM();
            //            printf("close\n");
            if((currRPM.w1==0)&&(currRPM.w2==0)&&currRPM.w3==0){
                sleep(1);
                break;
            }
            sleep(1);
            continue;
        }
        // Get Command
        if(Node.getNodeFlag()){
            count_cmd = 0;

            robotCMD = Node.getMotion();

            Base.Send(robotCMD);
#ifdef DEBUG_
            printf("\n*****get motion******\n");
            Node.ShowCommand();
#endif
        }else{
            if(count_cmd>=CMD_FREQUENCY/2){
                printf("CANNOT GET COMMAND %d\n", count_cmd);
                Base.Close();
            }else{
                count_cmd++;
            }
        }
        // Get feedback
        if(Base.GetBaseFlag()){
            count_fb = 0;
            currRPM = Base.GetCurrRPM();
            Node.pub_robotFB(Base.GetOdometry());
#ifdef DEBUG_
            printf("\n*****get feedback******\n");
            printf("motor1 rpm %f\nmotor2 rpm %f\nmotor3 rpm %f\n", currRPM.w1, currRPM.w2, currRPM.w3);
#endif
        }else{
            if(count_fb>=CMD_FREQUENCY/2){
                printf("CANNOT GET FEETBACK %d\n", count_fb);
            }else{
                count_fb++;
            }
        }
       
	ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Close Attack Motion\n";
    return 0;
}


