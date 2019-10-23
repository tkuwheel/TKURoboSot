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
#include "node_handle.h"
#include "base_control.h"
/********************************
 *	Define	
 ********************************/
//typedef void * (*THREADFUNCPTR)(void *);
//#define DEBUG
bool flag = 0;
void inturrupt(int signal)
{
    printf("get SIGINT %d\n", signal);
    flag = 1;
}
int main(int argc, char **argv)
{
    if(argc >= 2){
        if(strcmp(argv[1],"-D")==0){
            printf("hi\n");
        }
    }
    Motion_nodeHandle Node(argc, argv);
    //    ros::init(argc, argv, "Test");
    //    ros::NodeHandle n;
    BaseController Base(argc, argv, false);

    RobotCommand robotCMD={0};
    RobotCommand robotOdo={0};
    MotorSpeed currRPM;
    signal(SIGINT, inturrupt);
    printf("\033[1;32m***FIRA7 IS RUNNING!***\n\033[0;37m");
    ros::Rate loop_rate(CMD_FREQUENCY);
    unsigned int counter_cmd = 0;
    unsigned int counter = 0;
    unsigned int counter_shoot = 0;
    bool no_cmd = false;
    bool cmd = false;
    struct timeval last, now;
    gettimeofday(&last, 0);
    while(true){
        // close 
        if(flag){
            Base.Close();
            Base.ShowCsslCallback();
            currRPM = Base.GetCurrRPM();
            if((currRPM.w1==0)&&(currRPM.w2==0)&&currRPM.w3==0){
                sleep(1);
                break;
            }
            sleep(1);
            continue;
        }
        // Get command
        if(Node.getMotionFlag()){
            counter_cmd = 0;
            robotCMD = Node.getMotion();
            Base.Send(robotCMD);
            if(robotCMD.shoot_power>0){
//                gettimeofday(&now, 0);
//                int s = (now.tv_sec - last.tv_sec)*1000;
//                double us = (now.tv_usec - last.tv_usec)/1000;
//                printf("%d shoot %f\n", counter_shoot++, (double)s+us);
//                fflush(stdout); 
//                last = now;
                Node.clearShoot();
            }
            no_cmd = false;
            if(!cmd){
                cmd = true;
                counter++;
                printf("\033[1;32m\nFIRA7 GET COMMAND--- %d\n\033[0;37m", counter);
            }
#ifdef DEBUG
            printf("\n*****get motion******\n");
            Node.ShowCommand();
            Base.ShowCsslSend();
#endif
        }else{
            if(counter_cmd >= (CMD_FREQUENCY/2)){
                counter_cmd = 0;
                Node.clearAll();
                Base.Close();
                cmd = false;
                if(!no_cmd){
                    no_cmd = true;
                    counter++;
                    printf("\033[0;33m\nFIRA7 CANNOT GET COMMAND--- %d\n\033[0;37m", counter);
                }
            }else{
                counter_cmd++;
            }
        }
        // Get feedback
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
    std::cout << "\033[0;32mClose FIRA7 Motion\n";
    return 0;
}


