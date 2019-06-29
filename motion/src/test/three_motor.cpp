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

    if(argc == 4){
        int16_t pwm1 = strtol(argv[1], NULL, 10);
        int16_t pwm2 = strtol(argv[2], NULL, 10);
        int16_t pwm3 = strtol(argv[3], NULL, 10);
        ros::init(argc, argv, "Three");
        ros::NodeHandle n;
        BaseControl Base(argc, argv, true);


        serial_rx RX;
//        serial_rx* RX = &rx;
        std::cout << "\t pwm1 " << pwm1 <<std::endl;
        std::cout << "\t pwm2 " << pwm2 <<std::endl;
        std::cout << "\t pwm3 " << pwm3 <<std::endl;

        signal(SIGINT, inturrupt);
        ros::Rate loop_rate(1);
        Base.SetTriple(pwm1, pwm2, pwm3);
        double scale;
        double real_rpm1, real_rpm2, real_rpm3;
        double target_rpm1, target_rpm2, target_rpm3;
        while(true){
            if(flag){
                break;
            }
            Base.SetTriple(pwm1, pwm2, pwm3);
            if(Base.GetBaseFlag()){
                RX = Base.GetOdoMotor();
#ifdef DEBUG
                scale = 1/(0.000001 * RX.duration);
                real_rpm1 = RX.w1 * scale * 60 / 2000;
                real_rpm2 = RX.w2 * scale * 60 / 2000;
                real_rpm3 = RX.w3 * scale * 60 / 2000;
                target_rpm1 = (fabs(pwm1) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
                target_rpm2 = (fabs(pwm2) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
                target_rpm3 = (fabs(pwm3) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
                printf("\n*****motor command******\n");

                printf("motor1 target pwm: %d\t, target rpm: %f\n", pwm1, target_rpm1);
                printf("motor2 target pwm: %d\t, target rpm: %f\n", pwm2, target_rpm2);
                printf("motor3 target pwm: %d\t, target rpm: %f\n", pwm3, target_rpm3);
                printf("\n*****get feedback******\n");
                printf("id: %d\t", RX.id);
                printf("size: %d\t", RX.size);
                printf("duration: %d\t\n", (int)RX.duration);
                printf("w1 ticks: %d\t w1 rpm: %f\n", RX.w1, real_rpm1);
                printf("w2 ticks: %d\t w2 rpm: %f\n", RX.w2, real_rpm2);
                printf("w3 ticks: %d\t w3 rpm: %f\n", RX.w3, real_rpm3);
#endif
            }
            loop_rate.sleep();
        }
        Base.McsslFinish();
        std::cout << "Close Attack Motion\n";


    }else{
        std::cout << "usage: [motor1 rpm] [motor2 rpm] [motor3 rpm]\n";
    }
    return 0;
}
