#include <stdlib.h>
#include <iostream>
#include "base_control.h"
#include "ros/ros.h"

bool flag = 0;
void inturrupt(int signal)
{
    printf("get SIGINT %d\n", signal);
    flag = 1;
}
int main(int argc, char** argv)
{
    std::cout << "argc " << argc << std::endl;

    if(argc == 3){
        int number = strtol(argv[1], NULL, 10);
        int16_t rpm = strtol(argv[2], NULL, 10);
        ros::init(argc, argv, "Single");
        ros::NodeHandle n;
        BaseControl Base;


        serial_rx* RX;
        std::cout << "motor number: " << number;
        std::cout << "\t rpm " << rpm <<std::endl;

        signal(SIGINT, inturrupt);
        Base.SetSingle(number, rpm);
        ros::Rate loop_rate(1);
        while(true){
            if(flag){
                break;
            }
            if(Base.GetBaseFlag()){
                RX = Base.GetPack();
#ifdef DEBUG

                printf("\n*****motor command******\n");

                std::cout << std::dec;
                std::cout << "motor number: " << number << "\t";
                std::cout << "motor target rpm: " << rpm << "\t";
                std::cout << std::hex;
                std::cout << "motor target rpm: " << rpm << "\n";
                printf("\n*****get feedback******\n");
                std::cout << std::dec;
                std::cout << "id: " << RX->id << "\t";
                std::cout << "size: " << RX->size << "\t";
                std::cout << "duration: " << RX->duration << "\t\n";
//                std::cout << "w1: " << RX->w1 * 600  / 2000 << "\t";
//                std::cout << std::dec << RX->w1 * 600 / 2000 << "\n";
                std::cout << "w1: " << RX->w1 * FB_FREQUENCY * 60 / 2000<< "\t";
                std::cout << RX->w1 << "\n";
                std::cout << "w2: " << RX->w2 << "\t\n";
                std::cout << "w3: " << RX->w3 << "\t\n";
#endif
            }
            loop_rate.sleep();
        }
        Base.McsslFinish();
        std::cout << "Close Attack Motion\n";


    }else{
        std::cout << "usage: [motor number] [motor rpm]\n";
    }
    return 0;
}
