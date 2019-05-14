#include "base_control.h"

//#include "../common/cssl/cssl.h"
#include "cssl.c"
#include "port.h"
serial_rx* Base_Control::base_RX = NULL;
Base_Control::Base_Control()
{
	this->base_robotFB = new robot_command;
	this->base_robotFB->x_speed = new double;
	this->base_robotFB->y_speed = new double;
	this->base_robotFB->yaw_speed = new double;
	this->base_robotFB->shoot_power = new int;
	std::memset(this->base_robotFB->x_speed, 0, sizeof(double));
	std::memset(this->base_robotFB->y_speed, 0, sizeof(double));
	std::memset(this->base_robotFB->yaw_speed, 0, sizeof(double));
	std::memset(this->base_robotFB->shoot_power, 0, sizeof(int));

/*	Base_Control::base_RX = new serial_rx;

	Base_Control::base_RX->head1 = new unsigned char;
	Base_Control::base_RX->head2 = new unsigned char;
	Base_Control::base_RX->w1 = new int;
	Base_Control::base_RX->w2 = new int;
	Base_Control::base_RX->w3 = new int;
	Base_Control::base_RX->shoot = new unsigned char;
	Base_Control::base_RX->batery = new unsigned char;
	memset(Base_Control::base_RX->head1, 0, sizeof(unsigned char));
	memset(Base_Control::base_RX->head2, 0, sizeof(unsigned char));
	memset(Base_Control::base_RX->w1, 0, sizeof(int));
	memset(Base_Control::base_RX->w2, 0, sizeof(int));
	memset(Base_Control::base_RX->w3, 0, sizeof(int));
	memset(Base_Control::base_RX->shoot, 0, sizeof(unsigned char));
	memset(Base_Control::base_RX->batery, 0, sizeof(unsigned char));
*/
	this->base_TX = new serial_tx;

	this->base_TX->head1 = new unsigned char;
	this->base_TX->head2 = new unsigned char;
	this->base_TX->w1 = new unsigned char;
	this->base_TX->w2 = new unsigned char;
	this->base_TX->w3 = new unsigned char;
	this->base_TX->w4 = new unsigned char;
	this->base_TX->enable_and_stop = new unsigned char;
	this->base_TX->shoot = new unsigned char;
	this->base_TX->checksum = new unsigned char;

	memset(this->base_TX->head1, 0xff, sizeof(unsigned char));
	memset(this->base_TX->head2, 0xfa, sizeof(unsigned char));
	memset(this->base_TX->w1, 0, sizeof(unsigned char));
	memset(this->base_TX->w2, 0, sizeof(unsigned char));
	memset(this->base_TX->w3, 0, sizeof(unsigned char));
	memset(this->base_TX->w4, 0, sizeof(unsigned char));
	memset(this->base_TX->enable_and_stop, 0, sizeof(unsigned char));
	memset(this->base_TX->shoot, 0, sizeof(unsigned char));
	memset(this->base_TX->checksum, 0, sizeof(unsigned char));
	
	this->x_CMD = 0;
	this->y_CMD = 0;
	this->yaw_CMD = 0;
	this->serial = NULL;
#ifdef DEBUG
	std::cout << "Base_Control(DEBUG)\n";
	std::cout << "Init base control\n";
	/*std::cout << (int)*this->base_TX->head1 << std::endl;
	std::cout << (int)*this->base_TX->head2 << std::endl;
	std::cout << (int)*this->base_TX->w1 << std::endl;
	std::cout << (int)*this->base_TX->w2 << std::endl;
	std::cout << (int)*this->base_TX->w3 << std::endl;
	std::cout << (int)*this->base_TX->enable_and_stop << std::endl;
	std::cout << (int)*this->base_TX->shoot << std::endl;
	std::cout << (int)*this->base_TX->checksum << std::endl;*/
#endif
	mcssl_init();
}

Base_Control::~Base_Control()
{
#ifdef DEBUG
	std::cout << "~Base_Control(DEBUG)\n";
#endif
	mcssl_finish();
}

int Base_Control::mcssl_init()
{
	char *devs;
#ifdef DEBUG_CSSL
	std::cout << "mcssl_init(DEBUG_CSSL)\n";
#else
	cssl_start();
	std::cout << "test\n";
	if(!serial){
		devs = "/dev/ttyUSB0";
		serial = cssl_open(devs, /*mcssl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB1";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB2";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB3";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB4";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB5";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB6";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB7";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB8";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB9";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		devs = "/dev/ttyUSB10";
		serial = cssl_open(devs, /*msccl_Callback*/NULL, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		std::cout << cssl_geterrormsg() << std::endl;
		std::cout << "----> ATTACK MOTION RS232 OPEN FAILED <----\n";
		fflush(stdout);
		//return 0;
		std::cout << devs << std::endl;
		exit(EXIT_FAILURE);
	}else{
		std::cout << "----> ATTACK MOTION RS232 OPEN SUCCESSFUL <----\n";
		std::cout << "Initialize attack motion with port = "<< devs << "...\n";
		cssl_setflowcontrol(serial, 0, 0);
	}
	//serial->callback = &Base_Control::mcssl_Callback;
#endif
	return 1;
}

void Base_Control::mcssl_finish()
{
#ifdef DEBUG_CSSL
	std::cout << "mcssl_finish(DEBUG_CSSL)\n";
#else
	cssl_close(serial);
	cssl_stop();
#endif
}

void Base_Control::mcssl_Callback(int id, uint8_t* buf, int length)
{
#ifdef DEBUG_CSSLCALLBACK
	std::cout << "mcssl_Callback(DEBUG_CSSLCALLBACK)\n";
#else
#endif
}

void Base_Control::mcssl_send2motor()
{	
	*(this->base_TX->checksum) = *(this->base_TX->w1)+*(this->base_TX->w2)+*(this->base_TX->w3)+*(this->base_TX->w4);
#ifdef DEBUG_CSSL
	std::cout << "mcssl_send2motor(DEBUG_CSSL)\n";
	std::cout << std::hex;
	std::cout << "w1: " << (int)*(this->base_TX->w1) << std::endl;
	std::cout << "w2: " << (int)*(this->base_TX->w2) << std::endl;
	std::cout << "w3: " << (int)*(this->base_TX->w3) << std::endl;
	std::cout << "w4: " << (int)*(this->base_TX->w4) << std::endl;
	std::cout << "checksum: " << (int)*(this->base_TX->checksum) << std::endl;
	std::cout << std::endl;
#else
#ifdef DEBUG
	std::cout << "mcssl_send2motor(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "w1: " << (int)*(this->base_TX->w1) << std::endl;
	std::cout << "w2: " << (int)*(this->base_TX->w2) << std::endl;
	std::cout << "w3: " << (int)*(this->base_TX->w3) << std::endl;
	std::cout << "w4: " << (int)*(this->base_TX->w4) << std::endl;
	std::cout << "checksum: " << (int)*(this->base_TX->checksum) << std::endl;
	std::cout << std::endl;
#endif
	cssl_putchar(serial, *(this->base_TX->head1));
	cssl_putchar(serial, *(this->base_TX->head2));
	cssl_putchar(serial, *(this->base_TX->w1));
	cssl_putchar(serial, *(this->base_TX->w2));
	cssl_putchar(serial, *(this->base_TX->w3));
	cssl_putchar(serial, *(this->base_TX->w4));
	//cssl_putchar(serial, *(this->base_TX->checksum));
	/*cssl_putchar(serial, 0xff);
	cssl_putchar(serial, 0xfa);
	cssl_putchar(serial, 0);
	cssl_putchar(serial, 0);
	cssl_putchar(serial, 0);
	cssl_putchar(serial, 0x80);
	cssl_putchar(serial, 0x0);*/
#endif
}

void Base_Control::shoot_regularization()
{
	if(*(this->base_robotCMD->shoot_power)>=100){
		*(this->base_TX->shoot) = 255;
	}
	else{
		*(this->base_TX->shoot) = (255**(this->base_robotCMD->shoot_power)/100);
	}
#ifdef DEBUG
	std::cout << "shoot_regularization(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "shoot byte(hex): " << (int)*(this->base_TX->shoot) << std::endl;
	std::cout << std::endl;
#endif
}

void Base_Control::speed_regularization(double w1, double w2, double w3, double w4)
{
	unsigned char w1_dir = (w1<0)? 0x80 : 0;
	unsigned char w2_dir = (w2<0)? 0x80 : 0;
	unsigned char w3_dir = (w3<0)? 0x80 : 0;
	unsigned char w4_dir = (w4<0)? 0x80 : 0;

	double w1_speed_percent = (fabs(w1)<0.1)? 0 : fabs(w1);
	double w2_speed_percent = (fabs(w2)<0.1)? 0 : fabs(w2);
	double w3_speed_percent = (fabs(w3)<0.1)? 0 : fabs(w3);
	double w4_speed_percent = (fabs(w4)<0.1)? 0 : fabs(w4);

	if((w1_speed_percent>0.1) && (w1_speed_percent<5))w1_speed_percent=5;
	if((w2_speed_percent>0.1) && (w2_speed_percent<5))w2_speed_percent=5;
	if((w3_speed_percent>0.1) && (w3_speed_percent<5))w3_speed_percent=5;
	if((w4_speed_percent>0.1) && (w4_speed_percent<5))w4_speed_percent=5;

	if((w1_speed_percent>=100))w1_speed_percent=100;
	if((w2_speed_percent>=100))w2_speed_percent=100;
	if((w3_speed_percent>=100))w3_speed_percent=100;
	if((w4_speed_percent>=100))w4_speed_percent=100;

	*(this->base_TX->w1) = (w1_speed_percent>0)? (unsigned char)((127*w1_speed_percent/100) + w1_dir) : 0x80;
	*(this->base_TX->w2) = (w2_speed_percent>0)? (unsigned char)((127*w2_speed_percent/100) + w2_dir) : 0x80;
	*(this->base_TX->w3) = (w3_speed_percent>0)? (unsigned char)((127*w3_speed_percent/100) + w3_dir) : 0x80;
	*(this->base_TX->w4) = (w4_speed_percent>0)? (unsigned char)((127*w4_speed_percent/100) + w4_dir) : 0x80;
#ifdef DEBUG
	std::cout << "speed_regularization(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "motor1 speed(hex): " << (int)*(this->base_TX->w1) << std::endl;
	std::cout << "motor2 speed(hex): " << (int)*(this->base_TX->w2) << std::endl;
	std::cout << "motor3 speed(hex): " << (int)*(this->base_TX->w3) << std::endl;
	std::cout << "motor4 speed(hex): " << (int)*(this->base_TX->w4) << std::endl;
	std::cout << std::hex;
	std::cout << std::endl;

#endif
}

void Base_Control::forwardKinematics()
{

}

void Base_Control::inverseKinematics()
{
	double w1_speed, w2_speed, w3_speed, w4_speed;
	double x_error = *(this->base_robotCMD->x_speed) - x_CMD;
	double y_error = *(this->base_robotCMD->y_speed) - y_CMD;
	double yaw_error = *(this->base_robotCMD->yaw_speed) - yaw_CMD;
	if(x_error >= 0) x_CMD = (x_error>4)? x_CMD+4 :  *(this->base_robotCMD->x_speed);
	else x_CMD = (x_error<(-4))? x_CMD-4 :  *(this->base_robotCMD->x_speed);
	if(y_error >= 0) y_CMD = (y_error>4)? y_CMD+4 :  *(this->base_robotCMD->y_speed);
	else y_CMD = (y_error<(-4))? y_CMD-4 :  *(this->base_robotCMD->y_speed);
	if(yaw_error >= 0) yaw_CMD = (yaw_error>4)? yaw_CMD+4 :  *(this->base_robotCMD->yaw_speed);
	else yaw_CMD = (yaw_error<(-4))? yaw_CMD-4 :  *(this->base_robotCMD->yaw_speed);

	w1_speed = x_CMD*cos(m1_Angle)+y_CMD*sin(m1_Angle)+yaw_CMD*robot_radius*(-1);
	w2_speed = x_CMD*cos(m2_Angle)+y_CMD*sin(m2_Angle)+yaw_CMD*robot_radius*(-1);
	w3_speed = x_CMD*cos(m3_Angle)+y_CMD*sin(m3_Angle)+yaw_CMD*robot_radius*(-1);
	w4_speed = x_CMD*cos(m4_Angle)+y_CMD*sin(m4_Angle)+yaw_CMD*robot_radius*(-1);

	for(int i=0;i<10;i++){
		if(fabs(w1_speed)>100||fabs(w2_speed)>100||fabs(w3_speed>100)||fabs(w4_speed)>100){
			w1_speed = w1_speed*0.9;
			w2_speed = w2_speed*0.9;
			w3_speed = w3_speed*0.9;
			w4_speed = w4_speed*0.9;
		}else{
			w1_speed = w1_speed*7/5;
			w2_speed = w2_speed*7/5;
			w3_speed = w3_speed*7/5;
			w4_speed = w4_speed*7/5;
			break;
		}
	}
	speed_regularization(w1_speed, w2_speed, w3_speed, w4_speed);
#ifdef DEBUG
	std::cout << "Inverse kinematics(DEBUG)\n";
	std::cout << std::dec;
	std::cout << "x_speed CMD: " << *(base_robotCMD->x_speed) << std::endl;
	std::cout << "y_speed CMD: " << *(base_robotCMD->y_speed) << std::endl;
	std::cout << "yaw_speed CMD: " << *(base_robotCMD->yaw_speed) << std::endl;
	std::cout << "w1_speed(%): " << w1_speed << std::endl;
	std::cout << "w2_speed(%): " << w2_speed << std::endl;
	std::cout << "w3_speed(%): " << w3_speed << std::endl;
	std::cout << "w4_speed(%): " << w4_speed << std::endl;
	std::cout << std::endl;
#endif
}

void Base_Control::send(robot_command* CMD)
{
	this->base_robotCMD = CMD;
	inverseKinematics();
	mcssl_send2motor();
}

robot_command* Base_Control::get_feedback()
{
	forwardKinematics();
	return base_robotFB;
}
