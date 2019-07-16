#ifndef BaseControl_H
#define BaseControl_H
/*******************************
  * Include system header
  ******************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstring>
#include <ctime>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>
/*******************************
  * Include header
  ******************************/
//#include "motor_control.h"
#include "parameter.h"
#include "motor_data.h"
#include "cssl.h"
#include "crc_16.h"
/*******************************
  * Define 
  ******************************/
//#define DEBUG
#define RECORD
//#define CSSL
//#define DEBUG_CSSLCALLBACK
typedef void * (*THREADFUNCPTR)(void *);

typedef struct{
    uint8_t data[BUFFER_SIZE];
    int head, rear;
}SerialData;

typedef struct{
	uint8_t head1;
	uint8_t head2;
	uint8_t w1_h;
	uint8_t w1_l;
	uint8_t w2_h;
	uint8_t w2_l;
	uint8_t w3_h;
	uint8_t w3_l;
//	uint8_t w4;
	uint8_t enable_stop;
	uint8_t shoot;
	uint8_t crc_16_1;
	uint8_t crc_16_2;
	uint8_t checksum;
}SerialTX;

typedef struct{
	int id;
    int size;
    std::string error;
    int error_times;
    suseconds_t duration;
	int w1;
	int w2;
    int w3;
}SerialRX;

double PWM2RPM(const int16_t &);
int16_t RPM2PWM(const double &);

class BaseController{
public: //constructor & destructor
//    BaseControl();
    BaseController(int, char **, bool);
	~BaseController();
private: //const
	const double m1_Angle = -M_PI/6;
	const double m2_Angle = M_PI/6;
	const double m3_Angle = -M_PI;
//	const double robot_radius = 1;
	const double robot_radius = 0.15;
	const double wheel_radius = 0.0508;
    std::string port = "/dev/communication/motion";

public: //public function
    bool    GetBaseFlag();
    bool    GetErrFlag();
    uint8_t* GetPacket();
    MotorSpeed GetCurrRPM();
    MotorSpeed GetCurrPWM();
    MotorSpeed GetTarRPM();
    MotorSpeed GetTarPWM();
	void    Send(const RobotCommand &);
    void    SetSingle(int, int16_t);
    void    SetTriple(int16_t, int16_t, int16_t);
    void    SetEnable();
    void    SetStop();
    void    ClearOdo();
    void    Close();
//    RobotCommand GetOdoRobot();
//    RobotCommand GetTraj();
    void    ShowCsslSend();
    void    ShowCsslCallback();
    void    ShowCommand();
    void    ShowSerialPacket();
private: //private function
    static void *mpThreadRun(void *p);
    void    mRun();
	int 	mCsslInit();
    void 	mCsslFinish();
	void 	mCsslSend2FPGA();
	static void	mCsslCallback(int, uint8_t*, int);
    bool    mCheckSerial();
    bool    mSerialDecoder();
    void    mOpenRecordFile();
    void    mCloseRecordFile();
    void    mCommandRegularization();
    void    mSpeedRegularization();
    void    mSpeedRegularization(double &, const int &);
    int16_t mPWMRegularization(int16_t );
	void	mShootRegularization();
	void	mDriverSetting();
    void    mBaseControl();
    MotorSpeed  mTrapeziumSpeedPlan(const MotorSpeed &, const MotorSpeed &, MotorSpeed &);
    void    mSetSlope(const MotorSpeed &, const MotorSpeed &, int &, double []);
	void	mInverseKinematics();
	void	mForwardKinematics();	
    void    mTrajectory();
////    void    Filter();
////    void    MotorSpeed();
private: //private variable
    std::fstream fp;
    std::string record_name;
    pthread_t tid;
	cssl_t *serial;

    MotorSpeed m_motorCommand;
    MotorSpeed m_motorCommandRPM;
    MotorSpeed m_motorPreCmdCurrRPM;
    MotorSpeed m_motorCurrPWM;
    MotorSpeed m_motorTarPWM;
    MotorSpeed m_motorCurrRPM;
    MotorSpeed m_motorTarRPM;
	RobotCommand m_baseCommand;
	RobotCommand m_baseOdometry;
	SerialTX m_baseTX;
	SerialRX m_baseRX;
    bool mb_success;
    bool mb_record;
    bool mb_clear_odo;
    bool mb_base;
    bool mb_enable;
    bool mb_stop;
    bool mb_close;
    double m_duration;
    struct timeval m_serialNow;
    struct timeval m_serialLast;
    struct timeval m_commandTime;
    /* for serial callback*/
    static SerialData ms_serialData;
//    static bool msb_decoder;
    static bool msb_serial;
    static int  ms_length;

//    double  m_slope;
    int     m_interval;
    int     m_final_interval;
    double  m_slope[3];
    double  m_max_speed;
    uint8_t m_en_stop;
    uint8_t m_shoot_power;
    bool mb_hold_ball;
    bool mb_remote;

    Crc_16 Crc;
    uint8_t cssl_data[TX_PACKAGE_SIZE];
    uint8_t serial_data[RX_PACKAGE_SIZE];
    unsigned short crc_16;
};
#endif
