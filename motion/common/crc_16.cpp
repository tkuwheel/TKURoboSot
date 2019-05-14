

#include "crc_16.h"
#define POLY 0x1021

Crc_16::Crc_16(unsigned char* msg, int size){
    /*
         Constructure
         Arg:
            msg: input data
            size: The size of input data (Byte)
    */
    crc = genCrc(msg, size);
}
Crc_16::~Crc_16(){
    // deconstructor do nothing 
};
unsigned short Crc_16::genCrc(unsigned char* msg, int size){
    /*
         Generate CRC-16-CCITT code 
         Arg:
            msg: input data
            size: The size of input data (Byte)
    */
    unsigned short crc(0x0000);
    
    while(size--){
        crc = crc16_table[(crc >> 8 ^ *msg++) & 0xff] ^ (crc << 8);
    }
    return crc;
};
unsigned short Crc_16::getCrc(){
    return crc;
};