#ifndef MotorControl_H
#define MotorControl_H
/*******************************
  * Include system header
  ******************************/
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <cstring>
/*******************************
  * Include header
  ******************************/
#include "parameter.h"
/*******************************
  * Define 
  ******************************/
//#define DEBUG
double  PWM2RPM(const int16_t &);
int16_t RPM2PWM(const double &);
class MotorController{
public:
	MotorController();
	~MotorController();
public:
public:
    double  MotorControl();
    void    SetSpeed(const double &, const double &);
    void    SetMaxRPM(const double &);
    void    SetPID(const double &, const double &, const double &);
    double  GetSinValue();
    double  GetTarRPM();
    double  GetCurrRPM();
    int16_t GetTarPWM();
    int16_t GetCurrPWM();
private:
    void    mPIDControl(const double &, const double &);
    void    mSpeedPlan(const double &, const double &);
private:
    int16_t m_tar_pwm;
    int16_t m_curr_pwm;

    double m_cmd_rpm;
    double m_tar_rpm;
    double m_curr_rpm;
    double m_acc_error;
    double m_pre_error;
    double m_sin_value; 
    double m_max_rpm;
    double m_kp;
    double m_ki;
    double m_kd;
};
#endif //MotorControl_H
