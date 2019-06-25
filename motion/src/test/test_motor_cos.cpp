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

    double pwm1 = 0, pwm2 = M_PI/4, pwm3 = M_PI/2;
    int16_t PWM1, PWM2, PWM3;
    ros::init(argc, argv, "Step");
    ros::NodeHandle n;
    BaseControl Base(argc, argv, true);


    serial_rx RX;
//    serial_rx* RX = &rx;

    signal(SIGINT, inturrupt);
    ros::Rate loop_rate(30);
    int count = 0;
    bool turn = true;
    double scale;
    double real_rpm1, real_rpm2, real_rpm3;
    double target_rpm1, target_rpm2, target_rpm3;
    while(true){
        if(flag){
            break;
        }
        PWM1 = (int16_t)(cos(pwm1)*(MAX_PWM-0.2*MAX_PWM));
        PWM2 = (int16_t)(cos(pwm2)*(MAX_PWM-0.2*MAX_PWM));
        PWM3 = (int16_t)(cos(pwm3)*(MAX_PWM-0.2*MAX_PWM));
        printf("%f %f %f %d %d %d\n", pwm1, pwm2, pwm3, PWM1, PWM2, PWM3);
        PWM1 = (PWM1>=0)? PWM1+MIN_PWM : PWM1-MIN_PWM;
        PWM2 = (PWM2>=0)? PWM2+MIN_PWM : PWM2-MIN_PWM;
        PWM3 = (PWM3>=0)? PWM3+MIN_PWM : PWM3-MIN_PWM;
        Base.SetTriple(PWM1, PWM2, PWM3);
        pwm1 += 0.01;
        pwm2 += 0.01;
        pwm3 += 0.01;
        if(pwm1 >= 2*M_PI)pwm1 = 0;
        if(pwm2 >= 2*M_PI)pwm2 = 0;
        if(pwm3 >= 2*M_PI)pwm3 = 0;
        count++;
        if(Base.GetBaseFlag()){
            RX = Base.GetOdoMotor();
#ifdef DEBUG
            scale = 1/(0.000001 * RX.duration);
            real_rpm1 = RX.w1 * scale * 60 / 2000;
            real_rpm2 = RX.w2 * scale * 60 / 2000;
            real_rpm3 = RX.w3 * scale * 60 / 2000;
            if(PWM1>=0){
                target_rpm1 = (PWM1 - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

            }else{
                target_rpm1 = -(fabs(PWM1) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

            }
            if(PWM2>=0){
                target_rpm2 = (PWM2 - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

            }else{
                target_rpm2 = -(fabs(PWM2) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

            }
            if(PWM3>=0){
                target_rpm3 = (PWM3 - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

            }else{
                target_rpm3 = -(fabs(PWM3) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);

            }
            printf("\n*****motor command******\n");

            printf("motor1 target pwm: %d\t, target rpm: %f\n", PWM1, target_rpm1);
            printf("motor2 target pwm: %d\t, target rpm: %f\n", PWM2, target_rpm2);
            printf("motor3 target pwm: %d\t, target rpm: %f\n", PWM3, target_rpm3);
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
    }
    Base.McsslFinish();
    std::cout << "Close Attack Motion\n";


    return 0;
}
