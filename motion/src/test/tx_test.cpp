#include <iostream>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include "cssl.h"


bool run = true;
void inturrupt(int signal)
{
    if(signal == 2)run=false;
}
int main(int argc, char** argv)
{
    signal(SIGINT, inturrupt);
    cssl_t *serial = NULL;
    std::string port = "/dev/communication/motion";
    cssl_start();
    if(!serial)
        serial = cssl_open(port.c_str(), NULL, 0, 115200, 8, 0, 1);
    if(!serial){
        std::cout << cssl_geterrormsg << std::endl;
        exit(EXIT_FAILURE);
    }else{
        cssl_setflowcontrol(serial, 0, 0);
    }
    std::cout << "TX Test running\n";
    uint8_t count1 = 0;
    uint8_t count2 = 16;
    uint8_t data[16];
    while(run){
        
        data[0] = count1;
        data[1] = count2;
        data[2] = count1;
        data[3] = count2;
        data[4] = count1;
        data[5] = count2;
        data[6] = count1;
        data[7] = count2;
        data[8] = count1;
        data[9] = count2;
        data[10] = count1;
        data[11] = count2;
        data[12] = count1;
        data[13] = count2;
        data[14] = count1;
        data[15] = count2;
        count1++;
        count2++;
        cssl_putdata(serial, data, 16);
        
        if(count1>=255)count1=0;
        if(count2>=255)count2=0;
        sleep(0.1);
    }


    return 0;
}
