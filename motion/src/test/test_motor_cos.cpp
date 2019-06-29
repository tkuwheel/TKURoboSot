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
    if(argc == 2){
        number = strtol(argv[1], NULL, 10);
    }else{
        std::cout << "usage: [motor number]\n";
        exit(EXIT_FAILURE);
    }
    double p = 0;
    int16_t pwm;
    int16_t P;
    ros::init(argc, argv, "Cos");
    ros::NodeHandle n;
    MotorController MC(argc, argv, true);

    signal(SIGINT, inturrupt);
    ros::Rate loop_rate(100);
    double real_rpm;
    double target_rpm;
    long duration;
    while(true){
        if(flag){
            break;
        }
        P = (int16_t)((cos(p))*100);
        if(P>0){
            pwm = P*(MAX_PWM-0.2*MAX_PWM)/100+MIN_PWM;
        }else if(P<0){
            pwm = P*(MAX_PWM-0.2*MAX_PWM)/100-MIN_PWM;
        }else{
            target_rpm = 0;
        }
        MC.SetSpeed(number, P);
        p += 0.02;
        if(p >= 2*M_PI)p = 0;
        if(MC.GetMotorFlag()){
            real_rpm = MC.GetSpeed();
            duration = MC.GetDuration();
#ifdef DEBUG
            if(P>0){
                target_rpm = (pwm - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

            }else if(P<0){
                target_rpm = -(fabs(pwm) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

            }else{
                target_rpm = 0;
            }
            printf("\n*****motor command******\n");

            printf("target pwm: %d(dec) %x(hex) target rpm: %f, p: %f\n", (int16_t)pwm, (int16_t)pwm, target_rpm, p);
            printf("\n*****get feedback******\n");
            printf("duration: %d\t\n", (int)duration);
                printf("motor speed(rpm): %f\n", real_rpm);
#endif
        }
        loop_rate.sleep();
    }
    MC.McsslFinish();
    std::cout << "Close Attack Motion\n";


    return 0;
}
