#include "motor_control.h"

MotorController::MotorController()
{
    m_counter = 0;
    m_tar_pwm = 0.0;
    m_curr_pwm = 0.0;
    m_cmd_rpm = 0.0;
    m_tar_rpm = 0.0;
    m_curr_rpm = 0.0;
    m_sin_value = 0.0;
    m_max_rpm = 0.0;
//    m_kp = 190.0;
//    m_ki = 1.0;
//    m_kd = 15.0;
    m_kd = 0;
    m_ki = 0;
    m_kd = 0;
    m_acc_error = 0.0;
    m_delta_error = 0.0;
    m_pre_error = 0.0;
}

MotorController::~MotorController()
{
#ifdef DEBUG
	std::cout << "\n~MotorController(DEBUG)\n";
#endif //DEBUG
}


void MotorController::mPIDControl(const double &tar_rpm, const double &curr_rpm)
{
    double error = (tar_rpm - curr_rpm)/60.0/FB_FREQUENCY;

    double delta_pwm = m_kp*error + m_ki*m_acc_error + kd*m_delta_error;
    m_curr_pwm += delta_pwm;
    if(m_curr_pwm >= MAX_PWM)
        m_curr_pwm = MAX_PWM;
    else if(m_curr_pwm <= -MAX_PWM)
        m_curr_pwm = -MAX_PWM;

    m_tar_rpm = tar_rpm;
}

void MotorController::mSpeedPlan(const double &cmd_rpm, const double &curr_rpm)
{
    m_sin_value += 0.01;
    if(m_sin_value >= 1)m_sin_value = 1;
    m_tar_rpm = m_sin_value*m_max_rpm;
    m_curr_pwm = m_tar_rpm/MAX_MOTOR_RPM * MAX_PWM;
}

int16_t MotorController::MotorControl()
{
    m_counter++;
    if(m_counter>=1){
        m_counter = 0;
        mSpeedPlan(m_cmd_rpm, m_curr_rpm);
    }
    return m_curr_pwm;
}

void MotorController::SetSpeed(const double &cmd_rpm, const double &curr_rpm)
{
    double error = (m_tar_rpm - curr_rpm)/60.0/FB_FREQUENCY;
    m_acc_error += error;
    m_delta_error += (error - m_pre_error)
    m_pre_error = error;
    m_max_rpm = cmd_rpm;
    m_cmd_rpm = cmd_rpm;
    m_curr_rpm = curr_rpm;
}

void MotorController::SetMaxRPM(const double &p)
{
//    m_max_rpm = p / 100 * MAX_MOTOR_RPM;
}

void MotorController::SetPID(const double &kp, const double &ki, const double &kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

double MotorController::GetSinValue()
{
    return m_sin_value;
}

double MotorController::GetTarRPM()
{
    return m_tar_rpm;
}

double MotorController::GetCurrRPM()
{
    return m_curr_rpm * CMD_FREQUENCY * 60;
}

int16_t MotorController::GetTarPWM()
{
    return m_tar_pwm;
}

int16_t MotorController::GetCurrPWM()
{
    return m_curr_pwm;
}

double PWM2RPM(const int16_t &pwm)
{
    if(pwm > 0) 
        return (pwm - MAX_PWM*0.1) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
    else if(pwm < 0)
        return (pwm + MAX_PWM*0.1) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
    else 
        return 0;
}

int16_t RPM2PWM(const double &rpm)
{
    if(rpm > 0) 
        return rpm * (MAX_PWM-MAX_PWM*0.2)/MAX_MOTOR_RPM + MAX_PWM*0.1;
    else if(rpm < 0)
        return rpm * (MAX_PWM-MAX_PWM*0.2)/MAX_MOTOR_RPM - MAX_PWM*0.1;
    else 
        return 0;
}
