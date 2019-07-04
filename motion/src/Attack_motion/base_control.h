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
#include "motor_control.h"
#include "parameter.h"
#include "motor_data.h"
#include "cssl.h"
//#include "port.h"
#include "crc_16.h"
/*******************************
  * Define 
  ******************************/
#define DEBUG
#define _RECORD
#define CSSL
//#define DEBUG_CSSLCALLBACK
typedef void * (*THREADFUNCPTR)(void *);

class BaseControl{
public:
	BaseControl(int, char**, bool);
	~BaseControl();

private:
	const double m1_Angle = -5*M_PI/6;
	const double m2_Angle = -M_PI/6;
	const double m3_Angle = M_PI;
//	const double robot_radius = 1;
	const double robot_radius = 0.15;
	const double wheel_radius = 0.0508;
    const char *port = "/dev/communication/motion";
//    const int package_size = RX_PACKAGE_SIZE;

    std::fstream fp1;
    std::fstream fp2;
    std::fstream fp3;
    std::string record_name1;
    std::string record_name2;
    std::string record_name3;
    pthread_t tid;
	cssl_t *serial;

	RobotCommand baseCommand;
	RobotCommand baseOdometry;
	SerialTX baseTX;
	SerialRX baseRX;
    bool record;
    bool clear_odo;
    bool base_flag;
    bool error_flag;
    bool clock;
    bool send_flag;
    struct timeval last_time;
    /* for serial callback*/
    static uint8_t serial_data[RX_PACKAGE_SIZE];
    static bool decoder_flag;
    static bool serial_flag;
    static int length;

    double x_CMD, y_CMD, yaw_CMD;
    int16_t tar_w1_pwm;
    int16_t tar_w2_pwm; 
    int16_t tar_w3_pwm;
    int16_t w1_pwm;
    int16_t w2_pwm; 
    int16_t w3_pwm;
    double w1_cos_value; 
    double w2_cos_value; 
    double w3_cos_value; 
    double tar_w1;
    double tar_w2;
    double tar_w3;
    double curr_w1;
    double curr_w2;
    double curr_w3;
    double w1_spd_err;
    double w2_spd_err;
    double w3_spd_err;
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
	void	DriverSetting(int16_t, int16_t, int16_t);
	void	SpeedRegularization(double, double, double);
	void	InverseKinematics();
	void	ForwardKinematics();	
    void    Trajectory();
    void    Run();
    void    FPGAInit();
    bool    SerialDecoder();
    void    Filter();
    void    SpeedControl();
    double  PWM2RPM(int16_t);
    int16_t RPM2PWM(double);
    double    CalSpdErr(double, double);    //calculate the motor speed error
public:
    static void *pThreadRun(void *p);
	void 	McsslFinish();
    bool    GetBaseFlag();
    bool    GetErrFlag();
    uint8_t* GetPacket();
    serial_rx GetOdoMotor();
    robot_command GetOdoRobot();
    robot_command GetTraj();
	void Send(const robot_command &);
    void SetSingle(int, int16_t);
    void SetTriple(int16_t, int16_t, int16_t);
    void SetClock();
};
#endif
