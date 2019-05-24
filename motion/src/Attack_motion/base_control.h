#ifndef BaseControl_H
#define BaseControl_H
/*******************************
  * Include system header
  ******************************/
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/time.h>
#include <cmath>
#include <cstring>
#include <ctime>
/*******************************
  * Include header
  ******************************/
#include "parameter.h"
#include "motor_data.h"
#include "cssl.h"
#include "port.h"
#include "crc_16.h"
/*******************************
  * Define 
  ******************************/
//#define DEBUG
//#define DEBUG_CSSL
#define DEBUG_CSSLCALLBACK
//typedef void * (*THREADFUNCPTR)(void *);

class BaseControl{
public:
	BaseControl();
	~BaseControl();

private:
	const double m1_Angle = -M_PI/6;
	const double m2_Angle = -5*M_PI/6;
	const double m3_Angle = M_PI/2;
	const double robot_radius = 1;
	//const double robot_radius = 0.15;
	const double wheel_radius = 0.0508;
	const double yaw_inv = 2.3251;
    const char *port = "/dev/communication/motion";
//    const int package_size = RX_PACKAGE_SIZE;

    RX_DATA_TYPE package_daga[RX_PACKAGE_SIZE];
    pthread_t tid;
	cssl_t *serial;

	robot_command base_robotCMD;
	robot_command base_robotFB;
	serial_tx base_TX;
	static serial_rx base_RX;
    static bool base_flag;
    static struct timeval last_time;

	double x_CMD, y_CMD, yaw_CMD;
	unsigned char en1,en2,en3,stop1,stop2,stop3;
    //unsigned char w1_dir,w2_dir,w3_dir;
	//static	unsigned char cssl_buffer[50];
	//static	int count_buffer;
	//void send();
	//void get();
private:
	static void	mcssl_Callback(int, uint8_t*, int);
	void 	mcssl_send2motor();
	int 	mcssl_init();
	void	shoot_regularization();
	void	speed_regularization(double, double, double);
	void	inverseKinematics();
	void	forwardKinematics();	
    void    run();
public:
    static void *pThreadRun(void *argv);
	void 	mcssl_finish();
    bool getBaseFlag();
    serial_rx* getPack();
	void send(const robot_command &);
	robot_command *get_feedback();
};
#endif
