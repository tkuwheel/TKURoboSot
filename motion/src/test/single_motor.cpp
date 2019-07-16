#include <stdlib.h>
#include <iostream>
#include "base_control.h"
#include "ros/ros.h"
#define DEBUG
bool flag = false;
void inturrupt(int signal)
{
    printf("get SIGINT %d\n", signal);
    flag = true;
}
int main(int argc, char** argv)
{
    std::cout << "argc " << argc << std::endl;

    int number;
    double rpm;
    if(argc == 3){
        number = strtol(argv[1], NULL, 10);
        rpm = strtof(argv[2], NULL);
    }else{
        std::cout << "usage: [motor number] [motor pwm]\n";
        exit(EXIT_FAILURE);
    }
    ros::init(argc, argv, "Single");
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
    Base.SetSingle(number, rpm);
    while(true){
        if(flag){
            Base.Close();
            printf("CLSOE\n");
            Base.ShowCsslCallback();
            currRPM = Base.GetCurrRPM();
            if((currRPM.w1==0)&&(currRPM.w2==0)&&currRPM.w3==0)
                break;
            loop_rate.sleep();
            continue;
        }
        Base.SetEnable();
        if(Base.GetBaseFlag()){
            currRPM = Base.GetCurrRPM();
            tarRPM = Base.GetTarRPM();
#ifdef DEBUG
//            printf("\n*****motor command******\n");

//            printf("motor number: %d\n", number);
//            Base.ShowCsslSend();
            printf("\n*****get feedback******\n");
            printf("motor1 rpm %f\nmotor2 rpm %f\nmotor3 rpm %f\n", currRPM.w1, currRPM.w2, currRPM.w3);
//            Base.ShowCsslCallback();
#endif
        }
        loop_rate.sleep();
    }
    std::cout << "Close Attack Motion\n";
    return 0;
}
