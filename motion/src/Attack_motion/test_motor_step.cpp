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


    serial_rx rx;
    serial_rx* RX = &rx;
    std::cout << "\t max pwm value " << max <<std::endl;

    signal(SIGINT, inturrupt);
    ros::Rate loop_rate(0.5);
    Base.SetTriple(max, max, max);
    while(true){
        if(flag){
            break;
        }
        Base.SetTriple(max, max, max);
        if(Base.GetBaseFlag()){
            rx = Base.GetOdo();
#ifdef DEBUG

            printf("\n*****motor command******\n");

            std::cout << std::dec;
            std::cout << "motor1 target pwm: " << max << "\t";
            std::cout << "motor2 target pwm: " << max << "\t";
            std::cout << "motor3 target pwm: " << max << "\n";
            std::cout << std::hex;
            std::cout << "motor1 target pwm: " << max << "\t";
            std::cout << "motor2 target pwm: " << max << "\t";
            std::cout << "motor3 target pwm: " << max << "\n";
            printf("\n*****get feedback******\n");
            std::cout << std::dec;
            std::cout << "id: " << RX->id << "\t";
            std::cout << "size: " << RX->size << "\t";
            std::cout << "duration: " << RX->duration << "\t\n";
            std::cout << "w1: " << RX->w1 * FB_FREQUENCY * 60 / 2000<< "\t";
            std::cout << RX->w1 << "\n";
            std::cout << "w2: " << RX->w2 * FB_FREQUENCY * 60 / 2000<< "\t";
            std::cout << RX->w2 << "\n";
            std::cout << "w3: " << RX->w3 * FB_FREQUENCY * 60 / 2000<< "\t";
            std::cout << RX->w3 << "\t\n";
            std::cout << RX->error << "\t\n";
#endif
        }
        loop_rate.sleep();
        max = -max;
    }
    Base.McsslFinish();
    std::cout << "Close Attack Motion\n";


    return 0;
}
