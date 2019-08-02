#include <stdlib.h>
#include <iostream>
#include "crc_16.h"

int main(int argc, char** argv)
{
    int size = argc - 1;
    std::cout << argc << std::endl;
    if(argc > 1){
//        char *pEnd;
//        size = strtol(argv[1], &pEnd, 10);
        uint8_t data[size];

        for(int i = 0; i<size; i++){
            data[i] = strtol(argv[i+1], NULL, 16);
            printf("%d %x\n", i, data[i]);
        }
        Crc_16 crc(data, size);
        unsigned short check = crc.getCrc();
        std::cout << std::hex;
        std::cout << check << std::endl;

    }else{
        uint8_t data[16];
        data[0] = 0xff;
        data[1] = 0xfa;
        data[2] = 0x0;
        data[3] = 0x0;
        data[4] = 0x0;
        data[5] = 0x0;
        data[6] = 0x0;
        data[7] = 0x0;
        data[8] = 0x0;
        data[9] = 0x1;
        data[10] = 0xff;
        data[11] = 0xff;
        data[12] = 0xff;
        data[13] = 0xf3;
        data[14] = 0x1b;
        data[15] = 0xb6;
        int size = sizeof(data)/sizeof(uint8_t);
        for(int i = 0; i<size; i++){
            printf("%d %x\n", i, data[i]);
        }
        Crc_16 crc(data, size);
        unsigned short check = crc.getCrc();

        std::cout << check << std::endl;

    }
    return 0;
}
