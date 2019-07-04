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
#define RECORD
#define CSSL
//#define DEBUG_CSSLCALLBACK
typedef void * (*THREADFUNCPTR)(void *);
typedef struct{
    uint8_t data[BUFFER_SIZE];
    int head, rear;
}SerialData;
typedef struct{
    int w1;
    int w2;
    int w3;
}MotorSpeed;
class MotorController{
public:
	MotorController(int, char**, bool, int number, std::string name);
//	MotorController(int, char**, bool);
	~MotorController();

private:
    int m_number;
    std::string m_name;
    const std::string port = "/dev/communication/motion";

    std::fstream fp;
    std::string record_name;
    pthread_t tid;
	cssl_t *serial;

	SerialTX motor_TX;
	SerialRX motor_RX;
//    MotorSpeed motorCommand;
    MotorSpeed motorPWM;
	double motor_odometry;
    long duration;
    bool record;
    bool motor_flag;
    bool decoder_error;
    bool bdecoder_success;
//    bool bsend;
    bool benable;
    bool clear_odo;
    bool close;
    bool get_target;
    struct timeval last_time;
    struct timeval now;
    /* for serial callback*/
//    static SerialData serialData;
    static SerialData serialData;
    static bool decoder_flag;
    static bool serial_flag;
    static int length;
//    static int rear;
//    static int head;

//    int16_t w1_pwm, w2_pwm, w3_pwm;
    int *pmotor_feedback;
    int *pmotor_pwm;
    int16_t tar_pwm;
    int16_t curr_pwm;
    double tar_rpm;
    double curr_rpm;
    double avg_rpm;
    double sin_value; 
    double error_rpm;
    double pre_error_rpm;
//    double delta_error_rpm;
	bool motor_enable;
    bool motor_stop;
    int max_rpm;

    Crc_16 Crc;
//    uint8_t crc_data[TX_PACKAGE_SIZE-2];
    uint8_t cssl_data[TX_PACKAGE_SIZE];
    uint8_t serial_data[RX_PACKAGE_SIZE];
    unsigned short crc_16;
private:
	static void	McsslCallback(int, uint8_t*, int);
	int 	McsslInit();
	void 	McsslSend2FPGA();
    void    McsslSend2FPGA(bool, bool, bool, bool, bool, bool); 
	void	DriverSetting();
    void    DriverSetting(int16_t, int16_t, int16_t);
    bool    SerialDecoder();
//	void	SpeedRegularization(double, double, double);
    void    Run();
    void    FPGAInit();
    void    Filter();
    void    SpeedControl();
    double  PWM2RPM(int16_t);
    int16_t RPM2PWM(double);
    double  SpdErr(double, double);    //calculate the motor speed error
    bool    GetDecoderErrFlag();
    double  GetSinValue();
public:
    static void *pThreadRun(void *p);
	void 	McsslFinish();

    bool    GetMotorFlag();
    double  GetTarRPM();
    double  GetCurrRPM();
    double  GetOdometry();
    int16_t GetTarPWM();
    int16_t GetCurrPWM();
    long    GetDuration();
    bool    GetMotorEnable(){return motor_enable;}
    bool    GetMotorStop(){return motor_stop;}
    int     GetNumber(){return m_number;}
    std::string GetName(){return m_name;}
    uint8_t* GetPacket();

    void SetMotor(int, std::string);
    void SetSpeed(double);
    void SetTarRPM(int);
    void SetEnable();
    void ClearOdo();
    void Close();
};
#endif //MotorControl_H
