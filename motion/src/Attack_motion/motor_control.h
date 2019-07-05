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
class MotorController{
public:
	MotorController();
	~MotorController();

private:
//    int m_number;
//    std::string m_name;
    int16_t m_tar_pwm;
    int16_t m_curr_pwm;
    double m_tar_rpm;
    double m_curr_rpm;
    double m_sin_value; 
    double m_max_rpm;

private:
    double  PWM2RPM(const int16_t &);
    int16_t RPM2PWM(const double &);
    void    SpeedControl(const double &, const double &);
public:

    void    SpeedControl();

    void    SetSpeed(const double &, const double &);
    void    SetTarRPM(const double &);
    void    SetCurrRPM(const double &);
    void    SetMaxRPM(const double &);

    double  GetSinValue();
    double  GetTarRPM();
    double  GetCurrRPM();
    int16_t GetTarPWM();
    int16_t GetCurrPWM();
//    int     GetNumber(){return m_number;}
//    std::string GetName(){return m_name;}
};
#endif //MotorControl_H
