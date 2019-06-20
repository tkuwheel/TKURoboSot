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
        int16_t rpm1 = strtol(argv[1], NULL, 10);
        int16_t rpm2 = strtol(argv[2], NULL, 10);
        int16_t rpm3 = strtol(argv[3], NULL, 10);
        ros::init(argc, argv, "Three");
        ros::NodeHandle n;
        BaseControl Base;


        serial_rx rx;
        serial_rx* RX = &rx;
        std::cout << "\t rpm1 " << rpm1 <<std::endl;
        std::cout << "\t rpm2 " << rpm2 <<std::endl;
        std::cout << "\t rpm3 " << rpm3 <<std::endl;

        signal(SIGINT, inturrupt);
        ros::Rate loop_rate(1);
        Base.SetTriple(rpm1, rpm2, rpm3);
        while(true){
            if(flag){
                break;
            }
            Base.SetTriple(rpm1, rpm2, rpm3);
            if(Base.GetBaseFlag()){
                rx = Base.GetOdo();
#ifdef DEBUG

                printf("\n*****motor command******\n");

                std::cout << std::dec;
                std::cout << "motor target rpm1: " << rpm1 << "\t";
                std::cout << "motor target rpm2: " << rpm2 << "\t";
                std::cout << "motor target rpm3: " << rpm3 << "\n";
                std::cout << std::hex;
                std::cout << "motor target rpm1: " << rpm1 << "\t";
                std::cout << "motor target rpm2: " << rpm2 << "\t";
                std::cout << "motor target rpm3: " << rpm3 << "\n";
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
        }
        Base.McsslFinish();
        std::cout << "Close Attack Motion\n";


    }else{
        std::cout << "usage: [motor1 rpm] [motor2 rpm] [motor3 rpm]\n";
    }
    return 0;
}
