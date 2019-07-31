#ifndef MOTOR_DATA_H
#define MOTOR_DATA_H
#include <stdint.h>
#include <sys/time.h>
#include <string>

typedef struct{
	double x;
	double y;
	double yaw;
	uint8_t shoot_power;
    bool hold_ball;
    bool remote;
}RobotCommand;

typedef struct{
    double w1;
    double w2;
    double w3;
}MotorSpeed;

typedef struct{
    RobotCommand vel;
    RobotCommand traj;
}Odo;


//typedef struct MOTOR_FEEDBACK motor_feedback;
//typedef struct MOTOR_COMMAND motor_command;
#endif 

