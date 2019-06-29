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
void CallBack(int id, uint8_t *buf, int len)
{
    printf("id: %d\t lenght: %d\n", id, len);
    printf("buf: ");
    for(int i = 0; i < len; i++){
        printf("%x ", *(buf + i));
    }
    printf("\n");
}
int main(int argc, char** argv)
{
    signal(SIGINT, inturrupt);
    cssl_t *serial = NULL;
    std::string port = "/dev/ttyUSB0";
    cssl_start();
    if(!serial)
        serial = cssl_open(port.c_str(), CallBack, 0, 115200, 8, 0, 1);
    if(!serial){
        std::cout << cssl_geterrormsg << std::endl;
        exit(EXIT_FAILURE);
    }else{
        cssl_setflowcontrol(serial, 0, 0);
    }
    std::cout << "RX Test running\n";
    uint8_t count1 = 0;
    uint8_t count2 = 16;
    uint8_t data[2];
    while(run){
    }


    return 0;
}
