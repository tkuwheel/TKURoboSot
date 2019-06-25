#ifndef BaseControl_H
#define BaseControl_H
/*******************************
  * Include system header
  ******************************/
#include <iostream>
#include <fstream>
#include <sstream>
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
#define CSSL
//#define DEBUG_CSSLCALLBACK
typedef void * (*THREADFUNCPTR)(void *);

class BaseControl{
public:
	BaseControl();
	BaseControl(int, char**, bool);
	~BaseControl();

private:
	const double m1_Angle = -M_PI/6;
	const double m2_Angle = -5*M_PI/6;
	const double m3_Angle = M_PI/2;
//	const double robot_radius = 1;
	const double robot_radius = 0.15;
	const double wheel_radius = 0.0508;
	const double yaw_inv = 2.3251;
    const char *port = "/dev/communication/motion";
//    const int package_size = RX_PACKAGE_SIZE;

    std::fstream fp;
    std::string record_name;
    RX_DATA_TYPE package_daga[RX_PACKAGE_SIZE];
    pthread_t tid;
	cssl_t *serial;

	robot_command base_robotCMD;
	robot_command odometry_robot;
	serial_tx base_TX;
	serial_rx base_RX;
	serial_rx odometry_motor;
    bool record;
    bool clear_odo;
    bool base_flag;
    bool error_flag;
    struct timeval last_time;
/* for serial callback*/
	static uint8_t serial_data[RX_PACKAGE_SIZE];
    static bool decoder_flag;
    static bool serial_flag;
    static int length;

	double x_CMD, y_CMD, yaw_CMD;
    int16_t w1, w2, w3;
    uint8_t shoot_power;
	bool en1,en2,en3,stop1,stop2,stop3;
    bool hold_ball;
    bool remote;
    bool enable_flag;

    Crc_16 Crc;
    unsigned short crc_16;
private:
	static void	McsslCallback(int, uint8_t*, int);
	void 	McsslSend2FPGA();
	int 	McsslInit();
	void	ShootRegularization();
	void	PWMRegularization(int, int, int);
	void	SpeedRegularization(double, double, double);
	void	InverseKinematics();
	void	ForwardKinematics();	
    void    ReEnable();
    void    Run();
    void    FPGAInit();
    bool    SerialDecoder();
public:
    static void *pThreadRun(void *p);
	void 	McsslFinish();
    bool    GetBaseFlag();
    bool    GetErrFlag();
    uint8_t* GetPacket();
    serial_rx GetOdoMotor();
    robot_command GetOdoRobot();
	void Send(const robot_command &);
    void SetSingle(int, int16_t);
    void SetTriple(int16_t, int16_t, int16_t);
};
#endif
