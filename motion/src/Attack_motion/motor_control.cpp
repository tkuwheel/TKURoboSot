#include "motor_control.h"

MotorController::MotorController()
{
    m_tar_pwm = 0;
    m_curr_pwm = 0;
    m_tar_rpm = 0;
    m_curr_rpm = 0;
    m_sin_value = 0;
    m_max_rpm = 0;
}

MotorController::~MotorController()
{
#ifdef DEBUG
	std::cout << "\n~MotorController(DEBUG)\n";
#endif //DEBUG
}


void MotorController::SpeedControl(const double &tar_rpm, const double &curr_rpm)
{
    double error_rpm = m_tar_rpm - m_curr_rpm;
    double k_p = 0.015;
    double k_i = 0 ;
    double k_d = 0;
    m_curr_pwm = RPM2PWM(sin(m_sin_value)*m_max_rpm);

    if(curr_rpm >= 0){
        if(error_rpm > 0 /*&& (fabs(error_rpm)>100)*/){
//            m_sin_value += (0.02 + acc_error_rpm);
            m_sin_value += (0.03);
            if(m_sin_value >= M_PI/2)m_sin_value = M_PI/2;
        }
        if(error_rpm < 0 /*&& (fabs(error_rpm)>100)*/){
//            m_sin_value += ( + acc_error_rpm);
            m_sin_value -= (0.02);
            if(m_sin_value <= -M_PI/2)m_sin_value = -M_PI/2;
        }

    }else if(curr_rpm < 0){
        if(error_rpm > 0 /*&& (fabs(error_rpm)>100)*/){
//            m_sin_value -= (0.01 + acc_error_rpm);
            m_sin_value += (0.02);
            if(m_sin_value >= M_PI/2)m_sin_value = M_PI/2;
        }
        if(error_rpm < 0 /*&& (fabs(error_rpm)>100)*/){
//            m_sin_value -= (0.02 + acc_error_rpm);
            m_sin_value -= (0.03);
            if(m_sin_value <= -M_PI/2)m_sin_value = -M_PI/2;
        }

    }
    if(fabs(curr_rpm)<=MIN_MOTOR_RPM && (tar_rpm * m_sin_value <=0)){
        m_sin_value = 0;
    }
//    pre_error_rpm = m_sin_value;

}

void MotorController::SpeedControl()
{
    SpeedControl(m_tar_rpm, m_curr_rpm);
}

void MotorController::SetSpeed(const double &p, const double &rpm)
{
    m_tar_rpm = p/100 * MAX_MOTOR_RPM;
    m_tar_pwm = RPM2PWM(m_tar_rpm);
    m_curr_rpm = rpm;
}

//void MotorController::SetTarRPM(const double &rpm)
//{
//    m_tar_rpm = rpm;
//    m_tar_pwm = RPM2PWM(rpm);
//}
//
//void MotorController::SetCurrRPM(const double &rpm)
//{
//    m_curr_rpm = rpm;
//}
//
void MotorController::SetMaxRPM(const double &p)
{
    m_max_rpm = p / 100 * MAX_MOTOR_RPM;
}

double MotorController::GetSinValue()
{
    return m_sin_value;
}

double MotorController::PWM2RPM(const int16_t &pwm)
{
    if(pwm > 0) 
        return (pwm - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
    else if(pwm < 0)
        return (pwm + MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
    else 
        return 0;
}

int16_t MotorController::RPM2PWM(const double &rpm)
{
    if(rpm > 0) 
        return rpm * (MAX_PWM-MAX_PWM*0.2)/MAX_MOTOR_RPM + MIN_PWM;
    else if(rpm < 0)
        return rpm * (MAX_PWM-MAX_PWM*0.2)/MAX_MOTOR_RPM - MIN_PWM;
    else 
        return 0;
}

double MotorController::GetTarRPM()
{
    return m_tar_rpm;
}

double MotorController::GetCurrRPM()
{
    return m_curr_rpm;
}

int16_t MotorController::GetTarPWM()
{
    return m_tar_pwm;
}

int16_t MotorController::GetCurrPWM()
{
    return m_curr_pwm;
}
