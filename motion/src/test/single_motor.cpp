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

    if(argc == 3){
        int number = strtol(argv[1], NULL, 10);
        double p = strtof(argv[2], NULL);
        int16_t pwm = p*(MAX_PWM-0.2*MAX_PWM)/100+MIN_PWM;
        ros::init(argc, argv, "Single");
        ros::NodeHandle n;
        MotorController Motor(argc, argv, true);


        std::cout << "motor number: " << number;
        std::cout << "\t speed " << p <<std::endl;

        signal(SIGINT, inturrupt);
        ros::Rate loop_rate(1);
        double real_rpm;
        double target_rpm;
        long duration;
        while(true){
            if(flag){
                break;
            }
            Motor.SetSpeed(number, p);
            if(Motor.GetMotorFlag()){
                duration = Motor.GetDuration();
                real_rpm = Motor.GetSpeed();
#ifdef DEBUG

                if(p>0){
                    target_rpm = (pwm - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

                }else if(p<0){
                    target_rpm = -(fabs(pwm) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

                }else{
                    target_rpm = 0;

                }
                printf("\n*****motor command******\n");

                printf("motor number: %d\t\n", number);
                printf("target pwm: %d(dec) %x(hex) target rpm: %f\n", (int16_t)pwm, (int16_t)pwm, target_rpm);
                printf("\n*****get feedback******\n");
                printf("duration: %d\t\n", (int)duration);
                printf("motor speed(rpm): %f\n", real_rpm);
//                printf("error message: %s\n", RX.error);
#endif
            }
            loop_rate.sleep();
        }
        Motor.McsslFinish();
        std::cout << "Close Attack Motion\n";


    }else{
        std::cout << "usage: [motor number] [motor pwm]\n";
    }
    return 0;
}
