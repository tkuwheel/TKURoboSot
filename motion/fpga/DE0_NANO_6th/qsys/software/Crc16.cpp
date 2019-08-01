/*
 * Crc16.cpp
 *
 *  Created on: 2019/6/11
 *      Author: ICLab
 */

#include "Crc16.h"

Crc_16::Crc_16() {
	// TODO Auto-generated constructor stub

}

Crc_16::~Crc_16() {
	// TODO Auto-generated destructor stub
}

unsigned short Crc_16::getCrc(unsigned char* msg, int size)
{
    return genCrc(msg, size);
}

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
