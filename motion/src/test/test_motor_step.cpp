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

    int16_t max;
    int number;

    if(argc == 3){
        if(argv[1] == "-h"){
            printf("usage: [motor number][pwm percent]\n");
            exit(EXIT_FAILURE);
        }
        max = strtol(argv[2], NULL, 10);
        number = strtol(argv[1], NULL, 10);
    }else{
        printf("usage: [motor number][pwm percent]\n");
        exit(EXIT_FAILURE);
    }
    ros::init(argc, argv, "Step");
    ros::NodeHandle n;
    std::stringstream ss;
    ss << number;
    std::string name = "Motor" + ss.str();
    MotorController MC(argc, argv, true, number, name);


    std::cout << "\t max pwm value " << max <<std::endl;

    signal(SIGINT, inturrupt);
    ros::Rate loop_rate(100);
    int count = 0;
    bool turn = true;
    double real_rpm = 0;
    double target_rpm = 0;
    int16_t real_pwm = 0;
    int16_t target_pwm = 0;
    long duration = 0;

    while(true){
        if(flag){
            MC.Close();
            if(MC.GetMotorFlag())
                if(MC.GetCurrRPM() == 0)break;
            continue;
        }
        if((count % 10) == 0){
            MC.SetSpeed(max);
        }
        MC.SetEnable();
        count++;
        if(MC.GetMotorFlag()){
#ifdef DEBUG
            real_rpm = MC.GetCurrRPM();
            duration = MC.GetDuration();
            target_rpm = MC.GetTarRPM();
            real_pwm = MC.GetCurrPWM();
            target_pwm = MC.GetTarPWM();
            printf("\n*****motor command******\n");

            printf("motor: %d real pwm: %dtarget pwm: %d target rpm: %f\n", number, (int16_t)real_pwm, target_pwm, target_rpm);
            printf("\n*****get feedback******\n");
            printf("duration: %d\t\n", (int)duration);
                printf("motor speed(rpm): %f\n", real_rpm);
#endif
        }
        loop_rate.sleep();
        if(count > 300){
            count = 0;
            max = -max;
        }
//        if(max>300)turn = false;
//        if(max<80)turn = true;
//        if(turn)max = max + 1;
//        else max = max - 1;
    }
//    MC.McsslFinish();
    std::cout << "Close Attack Motion\n";


    return 0;
}
