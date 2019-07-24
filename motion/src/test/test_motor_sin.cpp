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
    if(argc == 2){
        number = strtol(argv[1], NULL, 10);
    }else{
        std::cout << "usage: [motor number]\n";
        exit(EXIT_FAILURE);
    }
    double p = 0;
    double P = 0;
    double pwm;
    ros::init(argc, argv, "Sin");
    ros::NodeHandle n;

    std::stringstream ss;
    ss << number;
    std::string name = "Motor" + ss.str();
    MotorController MC(argc, argv, true, number, name);

    signal(SIGINT, inturrupt);
    ros::Rate loop_rate(200);
    double real_rpm = 0;
    double target_rpm = 0;
    long duration = 0;
    
    bool turn = false;
    int counter = 0;
    while(true){
        if(flag){
            MC.Close();
            if(MC.GetMotorFlag())
                if(MC.GetCurrRPM() == 0)break;
            continue;
        }
        if(counter % 7 == 0){
            P = sin(p)*100;
            MC.SetSpeed(P);
            p+=0.03;
            counter = 0;
        }
        counter++;
        if(p>=2 * M_PI)p=0;

        MC.SetEnable();
        pwm = MC.GetTarPWM();

        if(MC.GetMotorFlag()){
#ifdef DEBUG
            real_rpm = MC.GetCurrRPM();
            duration = MC.GetDuration();
            target_rpm = MC.GetTarRPM();
            printf("\n*****motor command******\n");

            printf("target pwm: %d(dec) %x(hex) target rpm: %f, p: %f\n", (int16_t)pwm, (int16_t)pwm, target_rpm, p);
            printf("\n*****get feedback******\n");
            printf("duration: %d\t\n", (int)duration);
            printf("motor speed(rpm): %f\n", real_rpm);
#endif
        }
        loop_rate.sleep();
    }
//    MC.McsslFinish();
    std::cout << "Close Attack Motion\n";


    return 0;
}
