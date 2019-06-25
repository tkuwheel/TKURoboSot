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

    if(argc == 3){
        int number = strtol(argv[1], NULL, 10);
        int16_t pwm = strtol(argv[2], NULL, 10);
        ros::init(argc, argv, "Single");
        ros::NodeHandle n;
        BaseControl Base(argc, argv, true);


        serial_rx RX;
        std::cout << "motor number: " << number;
        std::cout << "\t pwm " << pwm <<std::endl;

        signal(SIGINT, inturrupt);
        Base.SetSingle(number, pwm);
        ros::Rate loop_rate(1);
        double scale;
        double real_rpm1, real_rpm2, real_rpm3;
        double target_rpm1, target_rpm2, target_rpm3, target_rpm;
        while(true){
            if(flag){
                break;
            }
            Base.SetSingle(number, pwm);
            if(Base.GetBaseFlag()){
                RX = Base.GetOdoMotor();
#ifdef DEBUG

                scale = 1/(0.000001 * RX.duration);
                real_rpm1 = RX.w1 * scale * 60 / 2000;
                real_rpm2 = RX.w2 * scale * 60 / 2000;
                real_rpm3 = RX.w3 * scale * 60 / 2000;
                if(pwm>=0){
                    target_rpm = (pwm - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

                }else{
                    target_rpm = -(fabs(pwm) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

                }
                printf("\n*****motor command******\n");

                printf("motor number: %d\t\n", number);
                printf("motor target pwm: %d\t, target rpm: %f\n", pwm, target_rpm);
                printf("motor target pwm: %x\t, target rpm: %f\n", pwm, target_rpm);
                printf("\n*****get feedback******\n");
                printf("id: %d\t", RX.id);
                printf("size: %d\t", RX.size);
                printf("duration: %d\t\n", RX.duration);
                printf("w1 ticks: %d\t w1 rpm: %f\n", RX.w1, real_rpm1);
                printf("w2 ticks: %d\t w2 rpm: %f\n", RX.w2, real_rpm2);
                printf("w3 ticks: %d\t w3 rpm: %f\n", RX.w3, real_rpm3);
//                printf("error message: %s\n", RX.error);
#endif
            }
            loop_rate.sleep();
        }
        Base.SetSingle(number, 0);
        Base.McsslFinish();
        std::cout << "Close Attack Motion\n";


    }else{
        std::cout << "usage: [motor number] [motor pwm]\n";
    }
    return 0;
}
