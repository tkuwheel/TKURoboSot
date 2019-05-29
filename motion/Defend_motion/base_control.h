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
	void	speed_regularization(double, double, double, double);
	void	inverseKinematics();
	void	forwardKinematics();	
private:
	const double m1_Angle =  M_PI/4;
	const double m2_Angle = (-1)*M_PI/4;
	const double m3_Angle = 3*M_PI/4;
	const double m4_Angle = (-1)*3*M_PI/4;

	//const double robot_radius = 0.2;
	const double robot_radius = 1;
	const double wheel_radius = 0.00508;

	cssl_t *serial;

	robot_command *base_robotCMD;
	robot_command *base_robotFB;
	serial_tx *base_TX;
	static serial_rx* base_RX;
	double x_CMD, y_CMD, yaw_CMD;
	unsigned char en1,en2,en3,stop1,stop2,stop3;
public:
	void send(robot_command*);
	robot_command* get_feedback();
//	int 	mcssl_init();
};
#endif
