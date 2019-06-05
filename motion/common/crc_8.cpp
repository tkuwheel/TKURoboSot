/** 
 * @brief crc-8 code generator and checker
 * 
 * @main crc_8.cpp
 * 
 * @author CHU, CHEN-YOU 
 **/ 

#include "crc_8.h"


Crc_8::Crc_8(unsigned char* msg, int size){
    crc = genCrc(msg, size);
};

Crc_8::~Crc_8(){
    // deconstructor do nothing 
};

unsigned char Crc_8::genCrc(unsigned char* msg, int size){
     unsigned char crc(0x00);
#ifdef METHOD1
     while(size--){
            crc ^= *msg++;
            for(int i=0;i<BYTE;i++){
                if(crc==0x80){
                    crc = (crc<<1)^POLY;
                }else{
                    crc = crc<<1;
                }
            }
     }
#endif
#ifdef METHOD2
	while(size--)
		crc = crc8_table[crc ^ *msg++];
#endif
    // printf("crc: %x\n", crc);
    return crc;
};

unsigned char Crc_8::genCrc(unsigned char msg){
     unsigned char crc(0x00);
    crc = msg;
    for(int i=8;i>0;i++){
        if(crc & 0x80){
            crc = (crc<<1)^POLY;
        }else{
            crc = (crc<<1);
        }
    }
    printf("crc: %x\n", crc);
    return crc;
};

unsigned char Crc_8::getCrc(){
    return crc;
};

bool Crc_8::checkCrc(unsigned char* msg, int size){
    unsigned char crc(genCrc(msg, size-1));
    if(!(crc-msg[size-1]))
        return true;
    else
        return false;
};

void Crc_8::buildCrcTable(void){
    for(int i=0;i<0xFF;i++){
        if(i%16==0)
            printf("\n");
        unsigned char j = i&0xFF;
        printf("0x%.2x, ", genCrc(j));
    }   
};