#ifndef Base_Control_H
#define Base_Control_H
/*******************************
  * Include system
  ******************************/
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <cstring>
/*******************************
  * Include library
  ******************************/
extern "C"{
#include "motor_data.h"
#include "cssl.h"
}
//#include "../common/cssl/cssl.c"
//#include "../common/cssl/port.h"

/*******************************
  * Define 
  ******************************/
//#define DEBUG
//#define DEBUG_CSSL
//#define DEBUG_CSSLCALLBACK
//#define DEBUG_CSSLCALLBACK_TEST

class Base_Control{
public:
	Base_Control();
	~Base_Control();

private:
	static void	mcssl_Callback(int, uint8_t*, int);
	void 	mcssl_finish();
	void 	mcssl_send2motor();
	int 	mcssl_init();
	void	shoot_regularization();
	void	speed_regularization(double, double, double);
	void	inverseKinematics();
	void	forwardKinematics();	
private:
	const double m1_Angle = -M_PI/3;
	const double m2_Angle =  M_PI/3;
	const double m3_Angle = -M_PI;
	const double robot_radius = 1;
	//const double robot_radius = 0.15;
	const double wheel_radius = 0.0508;
	const double yaw_inv = 2.3251;

	cssl_t *serial;
	//unsigned char w1_dir,w2_dir,w3_dir;
	//unsigned char w1_byte,w2_byte,w3_byte;
	//unsigned char en1,en2,en3,stop1,stop2,stop3;
	//unsigned char shoot_byte;

	robot_command *base_robotCMD;
	robot_command *base_robotFB;
	serial_tx *base_TX;
	static serial_rx* base_RX;

	double x_CMD, y_CMD, yaw_CMD;
	unsigned char en1,en2,en3,stop1,stop2,stop3;
//	unsigned char w1_dir,w2_dir,w3_dir;
	//static	unsigned char cssl_buffer[50];
	//static	int count_buffer;
	//void send();
	//void get();
public:
	void send(robot_command*);
	robot_command* get_feedback();
//	int 	mcssl_init();
};
#endif
