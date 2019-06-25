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

    int16_t max;

    if(argc == 2){
        if(argv[1] == "-h"){
            printf("usage: [pwm value]\n");
            exit(EXIT_FAILURE);
        }
        max = strtol(argv[1], NULL, 10);
        printf("pwm = %d/512\n", max);
    }else{
        max = 150;
        printf("pwm = %d/512\n", max);
    }
    ros::init(argc, argv, "Step");
    ros::NodeHandle n;
    BaseControl Base(argc, argv, true);


    serial_rx RX;
//    serial_rx* RX = &rx;
    std::cout << "\t max pwm value " << max <<std::endl;

    signal(SIGINT, inturrupt);
    ros::Rate loop_rate(30);
    Base.SetTriple(max, max, max);
    int count = 0;
    bool turn = true;
    double scale;
    double real_rpm1, real_rpm2, real_rpm3;
    double target_rpm1, target_rpm2, target_rpm3;
    while(true){
        if(flag){
            break;
        }
        Base.SetTriple(max, max, max);
        count++;
        if(Base.GetBaseFlag()){
            RX = Base.GetOdoMotor();
#ifdef DEBUG
            scale = 1/(0.000001 * RX.duration);
            real_rpm1 = RX.w1 * scale * 60 / 2000;
            real_rpm2 = RX.w2 * scale * 60 / 2000;
            real_rpm3 = RX.w3 * scale * 60 / 2000;
            if(max>=0){
                target_rpm1 = (max - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
                target_rpm2 = (max - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
                target_rpm3 = (max - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

            }else{
                target_rpm1 = -(fabs(max) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
                target_rpm2 = -(fabs(max) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
                target_rpm3 = -(fabs(max) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

            }
            printf("\n*****motor command******\n");

            printf("motor1 target pwm: %d\t, target rpm: %f\n", max, target_rpm1);
            printf("motor2 target pwm: %d\t, target rpm: %f\n", max, target_rpm2);
            printf("motor3 target pwm: %d\t, target rpm: %f\n", max, target_rpm3);
            printf("\n*****get feedback******\n");
            printf("id: %d\t", RX.id);
            printf("size: %d\t", RX.size);
            printf("duration: %d\t\n", RX.duration);
            printf("w1 ticks: %d\t w1 rpm: %f\n", RX.w1, real_rpm1);
            printf("w2 ticks: %d\t w2 rpm: %f\n", RX.w2, real_rpm2);
            printf("w3 ticks: %d\t w3 rpm: %f\n", RX.w3, real_rpm3);
#endif
        }
        loop_rate.sleep();
        if(count > 60){
            count = 0;
            max = -max;
        }
//        if(max>300)turn = false;
//        if(max<80)turn = true;
//        if(turn)max = max + 1;
//        else max = max - 1;
    }
    Base.McsslFinish();
    std::cout << "Close Attack Motion\n";


    return 0;
}
