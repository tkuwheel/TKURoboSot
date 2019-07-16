#include <stdlib.h>
#include <iostream>
#include "base_control.h"
#include "ros/ros.h"
//#define DEBUG
bool flag = false;
void inturrupt(int signal)
{
    printf("get SIGINT %d\n", signal);
    flag = true;
}
int main(int argc, char** argv)
{
    std::cout << "argc " << argc << std::endl;
    int16_t rpm1;
    int16_t rpm2;
    int16_t rpm3;

    if(argc == 4){
        rpm1 = strtol(argv[1], NULL, 10);
        rpm2 = strtol(argv[2], NULL, 10);
        rpm3 = strtol(argv[3], NULL, 10);
    }else{
        std::cout << "usage: [motor1 rpm] [motor2 rpm] [motor3 rpm]\n";
        exit(EXIT_FAILURE);
    }
    ros::init(argc, argv, "Triple");
    ros::NodeHandle n;
    BaseController Base(argc, argv, true);
    MotorSpeed currRPM;
    MotorSpeed tarRPM;
//
//
//    std::cout << "motor number: " << number;
//    std::cout << "\t speed " << p <<std::endl;
//
    signal(SIGINT, inturrupt);
    ros::Rate loop_rate(CMD_FREQUENCY);
//    ros::Rate loop_rate(1);
    double real_rpm;
    double target_rpm;
    long duration;
    int count = 0;
    Base.SetTriple(rpm1, rpm2, rpm3);
    while(true){
        if(flag){
            Base.Close();
            Base.ShowCsslCallback();
            currRPM = Base.GetCurrRPM();
            printf("close\n");
            if((currRPM.w1==0)&&(currRPM.w2==0)&&currRPM.w3==0)break;
            loop_rate.sleep();
            continue;
        }
        Base.SetEnable();
        count++;
        if(count>=8){
            count = 0;
            Base.SetTriple(rpm1, rpm2, rpm3);
        }
        if(Base.GetBaseFlag()){
            currRPM = Base.GetCurrRPM();
            tarRPM = Base.GetTarRPM();
#ifdef DEBUG
            printf("\n*****motor command******\n");

//            Base.ShowCsslSend();
            printf("\n*****get feedback******\n");
            printf("motor1 rpm %f\nmotor2 rpm %f\nmotor3 rpm %f\n", currRPM.w1, currRPM.w2, currRPM.w3);
//            Base.ShowCsslCallback();
#endif
        }
        loop_rate.sleep();
    }
    std::cout << "Close Attack Motion\n";
//    return 0;
}
