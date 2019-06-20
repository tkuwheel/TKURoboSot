#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include "cssl.h"


bool run = true;
void inturrupt(int signal)
{
    if(signal == 2)run=false;
}
int main(int argc, char* argv)
{
    signal(SIGINT, inturrupt);
    cssl_t *serial = NULL;
    char *port = "/dev/communication/motion";
    cssl_start();
    if(!serial)
        serial = cssl_open(port, NULL, 0, 115200, 8, 0, 1);
    if(!serial){
        std::cout << cssl_geterrormsg << std::endl;
        exit(EXIT_FAILURE);
    }else{
        cssl_setflowcontrol(serial, 0, 0);
    }
    std::cout << "TX Test running\n";
    uint8_t count1 = 0;
    uint8_t count2 = 16;
    uint8_t data[2];
    while(run){
        data[0] = count1++;
        data[1] = count2++;
        cssl_putdata(serial, data, 2);
        
        if(count1>=255)count1=0;
        if(count2>=255)count2=0;
        sleep(1);
    }


    return 0;
}
