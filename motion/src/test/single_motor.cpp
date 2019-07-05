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
    double p;
    if(argc == 3){
        number = strtol(argv[1], NULL, 10);
        p = strtof(argv[2], NULL);
    }else{
        std::cout << "usage: [motor number] [motor pwm]\n";
        exit(EXIT_FAILURE);
    }
//    int16_t pwm = p*(MAX_PWM-0.2*MAX_PWM)/100+MIN_PWM;
//
//    std::stringstream ss;
//    ss << number;
//    std::string name = "Motor" + ss.str();
    ros::init(argc, argv, "Single");
    ros::NodeHandle n;
    BaseController Base(argc, argv, false);
    MotorSpeed currRPM;
    MotorSpeed tarRPM;
//
//
//    std::cout << "motor number: " << number;
//    std::cout << "\t speed " << p <<std::endl;
//
    signal(SIGINT, inturrupt);
    ros::Rate loop_rate(1);
    double real_rpm;
    double target_rpm;
    long duration;
    while(true){
        if(flag){
            Base.Close();
            printf("CLSOE\n");
            if(Base.GetBaseFlag()){
                currRPM = Base.GetCurrRPM();
                if((currRPM.w1==0)&&(currRPM.w2==0)&&currRPM.w3==0)break;
            }
            continue;
        }
//        Motor.SetSpeed(p);
//        Motor.SetEnable();
        if(Base.GetBaseFlag()){
            currRPM = Base.GetCurrRPM();
//#ifdef DEBUG
//
//
//            targetRPM = Base.GetTarRPM();
            printf("\n*****motor command******\n");

//            printf("motor number: %d\n", number);
//            printf("target pwm: %d(dec) %x(hex) target rpm: %f\n", (int16_t)pwm, (int16_t)pwm, target_rpm);
            printf("\n*****get feedback******\n");
            Base.ShowCsslCallback();
//            printf("duration: %d\t\n", (int)duration);
//            printf("motor current speed(rpm): %f\n", currRPM.w1);
//            printf("motor current speed(rpm): %f\n", currRPM.w2);
//            printf("motor current speed(rpm): %f\n", currRPM.w2);
//            printf("error message: %s\n", RX.error);
//#endif
        }
        loop_rate.sleep();
    }
////    Motor.McsslFinish();
    std::cout << "Close Attack Motion\n";
//
//
//    return 0;
}
