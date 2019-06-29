#ifndef MOTOR_DATA_H
#define MOTOR_DATA_H
#include <stdint.h>
#include <sys/time.h>
#include <string>

#define TX_DATA_TYPE uint8_t 
#define RX_DATA_TYPE uint8_t 
#define MOTOR_DATA_TYPE int
//extern "C"{
typedef struct{
	double x;
	double y;
	double yaw;
	uint8_t shoot_power;
    bool hold_ball;
    bool remote;
}robot_command;

typedef struct{
    robot_command vel;
    robot_command traj;
}Odo;

typedef struct{
	TX_DATA_TYPE head1;
	TX_DATA_TYPE head2;
	TX_DATA_TYPE w1_h;
	TX_DATA_TYPE w1_l;
	TX_DATA_TYPE w2_h;
	TX_DATA_TYPE w2_l;
	TX_DATA_TYPE w3_h;
	TX_DATA_TYPE w3_l;
//	TX_DATA_TYPE w4;
	TX_DATA_TYPE enable_and_stop;
	TX_DATA_TYPE shoot;
	TX_DATA_TYPE crc_16_1;
	TX_DATA_TYPE crc_16_2;
	TX_DATA_TYPE checksum;
}serial_tx;

typedef struct{
	int id;
    int size;
    std::string error;
    suseconds_t duration;
	MOTOR_DATA_TYPE w1;
	MOTOR_DATA_TYPE w2;
    MOTOR_DATA_TYPE w3;
}serial_rx;

//typedef struct MOTOR_FEEDBACK motor_feedback;
//typedef struct MOTOR_COMMAND motor_command;
//}
#endif 

