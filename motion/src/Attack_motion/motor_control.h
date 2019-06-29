#ifndef MotorControl_H
#define MotorControl_H
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
#include "crc_16.h"
/*******************************
  * Define 
  ******************************/
//#define DEBUG
#define _RECORD
#define CSSL
//#define DEBUG_CSSLCALLBACK
typedef void * (*THREADFUNCPTR)(void *);

class MotorController{
public:
	MotorController();
	MotorController(int, char**, bool);
	~MotorController();

private:
    const char *port = "/dev/communication/motion";

    std::fstream fp;
    std::string record_name;
//    RX_DATA_TYPE package_data[RX_PACKAGE_SIZE];
    pthread_t tid;
	cssl_t *serial;

	serial_tx motor_TX;
	serial_rx motor_RX;
	int motor_odometry;
    long duration;
    bool record;
    bool motor_flag;
    bool error_flag;
    bool send_flag;
    bool clear_odo;
    struct timeval last_time;
    /* for serial callback*/
    static uint8_t serial_data[RX_PACKAGE_SIZE];
    static bool decoder_flag;
    static bool serial_flag;
    static int length;

    int16_t w1_pwm, w2_pwm, w3_pwm;
    int *pmotor_speed;
    int16_t tar_pwm;
    int16_t curr_pwm;
    double tar_rpm;
    double curr_rpm;
    double pre_rpm;
    double cos_value; 
    double spd_err;
	bool enable;
    bool stop;

    Crc_16 Crc;
    uint8_t crc_data[TX_PACKAGE_SIZE-2];
    uint8_t cssl_data[TX_PACKAGE_SIZE];
    unsigned short crc_16;
private:
	static void	McsslCallback(int, uint8_t*, int);
	int 	McsslInit();
	void 	McsslSend2FPGA();
    void    McsslSend2FPGA(
            bool, bool, bool, 
            bool, bool, bool); 
	void	DriverSetting();
    void    DriverSetting(int16_t, int16_t, int16_t);
    bool    SerialDecoder();
	void	SpeedRegularization(double, double, double);
    void    Run();
    void    FPGAInit();
    void    Filter();
    void    SpeedControl();
    double  PWM2RPM(int16_t);
    int16_t RPM2PWM(double);
    double  SpdErr(double, double);    //calculate the motor speed error
    bool    GetErrFlag();
public:
    static void *pThreadRun(void *p);
	void 	McsslFinish();
    bool    GetMotorFlag();
    uint8_t* GetPacket();
    double GetSpeed();
    long GetDuration();

    void SetSpeed(int, double);
};
#endif
