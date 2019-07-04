#include <stdlib.h>
#include <iostream>
#include "motor_control.h"
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
    int16_t pwm = p*(MAX_PWM-0.2*MAX_PWM)/100+MIN_PWM;

    std::stringstream ss;
    ss << number;
    std::string name = "Motor" + ss.str();
    ros::init(argc, argv, "Single");
    ros::NodeHandle n;
    MotorController Motor(argc, argv, true, number, name);


    std::cout << "motor number: " << number;
    std::cout << "\t speed " << p <<std::endl;

    signal(SIGINT, inturrupt);
    ros::Rate loop_rate(100);
    double real_rpm;
    double target_rpm;
    long duration;
    while(true){
        if(flag){
            Motor.Close();
            if(Motor.GetMotorFlag())
                if(Motor.GetCurrRPM() == 0)break;
            continue;
        }
        Motor.SetSpeed(p);
        Motor.SetEnable();
        if(Motor.GetMotorFlag()){
            duration = Motor.GetDuration();
            real_rpm = Motor.GetCurrRPM();
//            Motor.ClearOdo();
#ifdef DEBUG


            target_rpm = Motor.GetTarRPM();
            printf("\n*****motor command******\n");

            printf("motor number: %d\n", number);
            printf("target pwm: %d(dec) %x(hex) target rpm: %f\n", (int16_t)pwm, (int16_t)pwm, target_rpm);
            printf("\n*****get feedback******\n");
            printf("duration: %d\t\n", (int)duration);
            printf("motor current speed(rpm): %f\n", real_rpm);
            //                printf("error message: %s\n", RX.error);
#endif
        }
        loop_rate.sleep();
    }
//    Motor.McsslFinish();
    std::cout << "Close Attack Motion\n";


    return 0;
}
