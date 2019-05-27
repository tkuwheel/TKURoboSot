#include "base_control.h"

//#include "../common/cssl/cssl.h"
#include "cssl.c"
#include "port.h"
#include "crc_16.h"
serial_rx BaseControl::base_RX;
bool BaseControl::base_flag;
BaseControl::BaseControl()
{
    this->base_robotCMD = {0, 0, 0, 0, 0, 0};
    this->base_robotFB = {0, 0, 0, 0, 0};
    this->base_TX = {0xff, 0xfa, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    BaseControl::base_RX = {0, 0, 0, 0, 0, 0, 0};
    BaseControl::base_flag = false;
	this->serial = NULL;
#ifdef DEBUG
    std::cout << "BaseControl(DEBUG)\n";
#endif
#ifndef DEBUG_CSSL
	mcssl_init();
#endif
//    int p = pthread_create(&tid, NULL, &BaseControl::pThreadRun, this);
//    if(p != 0){
//        printf("base thread error\n");
//        exit(EXIT_FAILURE);
//    }
}

BaseControl::~BaseControl()
{
#ifdef DEBUG
	std::cout << "~BaseControl(DEBUG)\n";
#endif
	mcssl_finish();
}

int BaseControl::mcssl_init()
{
	std::cout << "==== Init cssl ====\n";
	cssl_start();
	if(!serial){
		serial = cssl_open(this->port, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	}
	if(!serial){
		std::cout << cssl_geterrormsg() << std::endl;
		std::cout << "===> ATTACK MOTION RS232 OPEN FAILED <===\n";
		fflush(stdout);
		//return 0;
		std::cout << this->port << std::endl;
		exit(EXIT_FAILURE);

	}else{
		std::cout << "----> ATTACK MOTION RS232 OPEN SUCCESSFUL <----\n";
		std::cout << "Initialize attack motion with port = "<< this->port << "...\n";
		cssl_setflowcontrol(serial, 0, 0);
	}
	return 1;
}

void BaseControl::mcssl_finish()
{
#ifdef DEBUG_CSSL
	std::cout << "mcssl_finish(DEBUG_CSSL)\n";
#else
	cssl_close(serial);
	cssl_stop();
#endif
}

void BaseControl::mcssl_Callback(int id, uint8_t* buf, int length)
{
//    for(int i = 0; i < length; i++){
//        printf("%x ", *(buf+i));
//    }
    int buffer_size = 50;
	static uint8_t cssl_buffer[50] = {0};
	static int count_buffer=0;
    for(int i = 0; i < length; i++){
	    count_buffer = (count_buffer)%100;
        cssl_buffer[count_buffer++] = *(buf+i);
    }
	int size = 16;
//    base_flag = false;
	for(int i=0; i<buffer_size; i++){
		if((cssl_buffer[i]==0xff)&&(cssl_buffer[(i+1)%buffer_size]==0xfa)){
			uint8_t data[] = {  cssl_buffer[i], cssl_buffer[(i+1)%buffer_size], cssl_buffer[(i+2)%buffer_size], 
                        cssl_buffer[(i+3)%buffer_size], cssl_buffer[(i+4)%buffer_size], cssl_buffer[(i+5)%buffer_size], 
                        cssl_buffer[(i+6)%buffer_size], cssl_buffer[(i+7)%buffer_size], cssl_buffer[(i+8)%buffer_size], 
                        cssl_buffer[(i+9)%buffer_size], cssl_buffer[(i+10)%buffer_size], cssl_buffer[(i+11)%buffer_size], 
                        cssl_buffer[(i+12)%buffer_size], cssl_buffer[(i+13)%buffer_size], cssl_buffer[(i+14)%buffer_size], 
                        cssl_buffer[(i+15)%buffer_size]};

            Crc_16 Crc16(data, size);
            unsigned short crc_16 = Crc16.getCrc();
//            printf("%x\n", crc_16);
			if(crc_16 == 0){
				base_RX.head1 = id;
				base_RX.head2 = length;
				base_RX.w1 = (cssl_buffer[(i+2)%buffer_size]<<24)+(cssl_buffer[(i+3)%buffer_size]<<16)+(cssl_buffer[(i+4)%buffer_size]<<8)+(cssl_buffer[(i+5)%buffer_size]);
				base_RX.w2 = (cssl_buffer[(i+6)%buffer_size]<<24)+(cssl_buffer[(i+7)%buffer_size]<<16)+(cssl_buffer[(i+8)%buffer_size]<<8)+(cssl_buffer[(i+9)%buffer_size]);
				base_RX.w3 = (cssl_buffer[(i+10)%buffer_size]<<24)+(cssl_buffer[(i+11)%buffer_size]<<16)+(cssl_buffer[(i+12)%buffer_size]<<8)+(cssl_buffer[(i+13)%buffer_size]);
				base_RX.shoot = 0;
				base_RX.batery = 0;
                base_flag = true;
				break;
			}else{
				continue;
			}
		}else{
			continue;
		}
	}
}

void BaseControl::mcssl_send2motor()
{	

    uint8_t data[] = {  this->base_TX.head1, 
                        this->base_TX.head2, 
                        this->base_TX.w1, 
                        this->base_TX.w2, 
                        this->base_TX.w3, 
                        this->base_TX.enable_and_stop, 
                        this->base_TX.shoot};

    int size = sizeof(data)/sizeof(uint8_t);
    Crc_16 Crc16(data, size);
    unsigned short crc_16 = Crc16.getCrc();
//    this->base_TX.crc_16_1 = *(unsigned char*)(&crc_16) + 1;
//    this->base_TX.crc_16_2 = *(unsigned char*)(&crc_16) + 0;
    this->base_TX.crc_16_1 = crc_16 >> 8;
    this->base_TX.crc_16_2 = crc_16;
    uint8_t buffer[] = {    this->base_TX.head1, 
                            this->base_TX.head2, 
                            this->base_TX.w1, 
                            this->base_TX.w2, 
                            this->base_TX.w3, 
                            this->base_TX.enable_and_stop, 
                            this->base_TX.shoot, 
                            this->base_TX.crc_16_1, 
                            this->base_TX.crc_16_2, 
                            0};
#ifndef DEBUG_CSSL
    cssl_putdata(serial, buffer, int(sizeof(buffer)/sizeof(uint8_t)));
#ifdef DEBUG
    printf("**************************\n");
    printf("* mcssl_send(DEBUG) *\n");
    printf("**************************\n");
    printf("already send to motor\n");
    printf("head1: %x\n", (this->base_TX.head1));
    printf("head2: %x\n", (this->base_TX.head2));
    printf("w1: %x\n", (this->base_TX.w1));
    printf("w2: %x\n", (this->base_TX.w2));
    printf("w3: %x\n", (this->base_TX.w3));
    printf("enable_and_stop: %x\n", (this->base_TX.enable_and_stop));
    printf("shoot: %x\n", (this->base_TX.shoot));
    printf("crc16-1: %x\n", (this->base_TX.crc_16_1));
    printf("crc16-2: %x\n", (this->base_TX.crc_16_2));
    printf("crc16: %x\n", (crc_16));
#endif
#endif
#ifdef DEBUG_CSSL
    printf("**************************\n");
    printf("* mcssl_send(DEBUG_CSSL) *\n");
    printf("**************************\n");
    printf("head1: %x\n", (this->base_TX.head1));
    printf("head2: %x\n", (this->base_TX.head2));
    printf("w1: %x\n", (this->base_TX.w1));
    printf("w2: %x\n", (this->base_TX.w2));
    printf("w3: %x\n", (this->base_TX.w3));
    printf("enable_and_stop: %x\n", (this->base_TX.enable_and_stop));
    printf("shoot: %x\n", (this->base_TX.shoot));
    printf("crc16-1: %x\n", (this->base_TX.crc_16_1));
    printf("crc16-2: %x\n", (this->base_TX.crc_16_2));
    printf("crc16: %x\n", (crc_16));
#endif
}

void BaseControl::shoot_regularization()
{
	if(this->base_robotCMD.shoot_power == 0){
		this->base_TX.shoot = 0;
	}else if(this->base_robotCMD.shoot_power >= 100){
		this->base_TX.shoot = 255;
	}
	else{
		this->base_TX.shoot = (255 * this->base_robotCMD.shoot_power / 100);
	}
#ifdef DEBUG
	std::cout << "shoot_regularization(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "shoot byte(hex): " << (int)(this->base_TX.shoot) << std::endl;
	std::cout << std::endl;
#endif
}

void BaseControl::speed_regularization(double w1, double w2, double w3)
{
	unsigned char w1_dir = (w1>0)? 0x80 : 0;
	unsigned char w2_dir = (w2>0)? 0x80 : 0;
	unsigned char w3_dir = (w3>0)? 0x80 : 0;

	double w1_speed_percent = (fabs(w1)<0.1)? 0 : fabs(w1);
	double w2_speed_percent = (fabs(w2)<0.1)? 0 : fabs(w2);
	double w3_speed_percent = (fabs(w3)<0.1)? 0 : fabs(w3);
	if(w1_speed_percent >= 100)w1_speed_percent = 100;
	if(w2_speed_percent >= 100)w2_speed_percent = 100;
	if(w3_speed_percent >= 100)w3_speed_percent = 100;
//	enable
	this->en1 = (w1_speed_percent > 0)? 1 : 0;
	this->en2 = (w2_speed_percent > 0)? 1 : 0;
	this->en3 = (w3_speed_percent > 0)? 1 : 0;
	this->stop1 = 0;
	this->stop2 = 0;
	this->stop3 = 0;
//	stop
/*
	if((w1_speed_percent != 999)&&(stop1 == 1)){
		en1 = 0;
	}else{
		en1 = ((stop1 == 1) || (w1_byte > 0))? 1 : 0;
	}
	if((w2_speed_percent != 999)&&(stop2 == 1)){
		en2 = 0;
	}else{
		en2 = ((stop2 == 1) || (w2_byte > 0))? 1 : 0;
	}
	if((w3_speed_percent != 999)&&(stop3 == 1)){
		en3 = 0;
	}else{
		en3 = ((stop3 == 1) || (w3_byte > 0))? 1 : 0;
	}
*/
//	speed -> speed_byte
	
	this->base_TX.w1 = (w1_speed_percent>0)? (unsigned char)((127*0.85*w1_speed_percent/100) + 12.7 + w1_dir) : 0x80;
	this->base_TX.w2 = (w2_speed_percent>0)? (unsigned char)((127*0.85*w2_speed_percent/100) + 12.7 + w2_dir) : 0x80;
	this->base_TX.w3 = (w3_speed_percent>0)? (unsigned char)((127*0.85*w3_speed_percent/100) + 12.7 + w3_dir) : 0x80;
    this->base_TX.enable_and_stop = (this->en1<<7) + (this->en2<<6) + (this->en3<<5) + (this->stop1<<4) + (this->stop2<<3) + (this->stop3<<2) + (this->base_robotCMD.hold_ball);
#ifdef DEBUG
	std::cout << "speed_regularization(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "motor1 speed(hex): " << (int)(this->base_TX.w1) << std::endl;
	std::cout << "motor2 speed(hex): " << (int)(this->base_TX.w2) << std::endl;
	std::cout << "motor3 speed(hex): " << (int)(this->base_TX.w3) << std::endl;
	std::cout << std::hex;
	std::cout << "enable & stop(hex): " << (int)(this->base_TX.enable_and_stop) << std::endl;
	std::cout << std::endl;
#endif
}

void BaseControl::forwardKinematics()
{
	double x,y;
	double yaw=0;
	int round=0;
	this->base_robotFB.x_speed = ( base_RX.w1 * (-0.3333) + base_RX.w2 * (-0.3333) + base_RX.w3 * (0.6667)) * 2 * M_PI * wheel_radius / 26 / 2000;
	this->base_robotFB.y_speed = ( base_RX.w1 * (0.5774) + base_RX.w2 * (-0.5774) + base_RX.w3 * (0)) * 2 * M_PI * wheel_radius / 26 / 2000;
	yaw = (base_RX.w1 * yaw_inv + base_RX.w2 * yaw_inv + base_RX.w3 * yaw_inv) * 2 * M_PI * wheel_radius / 2000 / 26;
	round = yaw/(2*M_PI);
	double yaw_degree;
	yaw_degree = (yaw - round*2*M_PI)*180/M_PI;
	if(yaw_degree>180){
		this->base_robotFB.yaw_speed = yaw_degree-360;
	}else if(yaw_degree<(-180)){
		this->base_robotFB.yaw_speed = yaw_degree+360;
	}else{
		this->base_robotFB.yaw_speed = yaw_degree;
	}
}

void BaseControl::inverseKinematics()
{
	double w1_speed, w2_speed, w3_speed;
//	double x_error = *(this->base_robotCMD->x_speed) - x_CMD;
//	double y_error = *(this->base_robotCMD->y_speed) - y_CMD;
//	double yaw_error = *(this->base_robotCMD->yaw_speed) - yaw_CMD;
    this->x_CMD = this->base_robotCMD.x_speed;
	this->y_CMD = this->base_robotCMD.y_speed;
	this->yaw_CMD = this->base_robotCMD.yaw_speed;

//
//	if(x_error>=0)x_CMD = (x_error>4)? x_CMD+4 : *(this->base_robotCMD->x_speed);
//	else x_CMD = (x_error<(-4))? x_CMD-4 : *(this->base_robotCMD->x_speed);
//	if(y_error>=0)y_CMD = (y_error>4)? y_CMD+4 : *(this->base_robotCMD->y_speed);
//	else y_CMD = (y_error<(-4))? y_CMD-4 : *(this->base_robotCMD->y_speed);
//	if(yaw_error>=0)yaw_CMD = (yaw_error>4)? yaw_CMD+4 : *(this->base_robotCMD->yaw_speed);
//	else yaw_CMD = (yaw_error<(-4))? yaw_CMD-4 : *(this->base_robotCMD->yaw_speed);
//
//	w1_speed = this->x_CMD*cos(m1_Angle)+y_CMD*sin(m1_Angle)+yaw_CMD*robot_radius*(-1);
//	w2_speed = this->x_CMD*cos(m2_Angle)+y_CMD*sin(m2_Angle)+yaw_CMD*robot_radius*(-1);
//	w3_speed = this->x_CMD*cos(m3_Angle)+y_CMD*sin(m3_Angle)+yaw_CMD*robot_radius*(-1);
//
//	for(int i=0;i<10;i++){
//		if(fabs(w1_speed)>100||fabs(w2_speed)>100||fabs(w3_speed)>100){
//			w1_speed = w1_speed*0.9;
//			w2_speed = w2_speed*0.9;
//			w3_speed = w3_speed*0.9;
//		}else{
//			w1_speed = w1_speed;
//			w2_speed = w2_speed;
//			w3_speed = w3_speed;
//			break;
//		}
//	}
	w1_speed = -this->x_CMD*cos(m1_Angle)+y_CMD*sin(m1_Angle)+yaw_CMD*robot_radius*(-1);
	w2_speed = -this->x_CMD*cos(m2_Angle)+y_CMD*sin(m2_Angle)+yaw_CMD*robot_radius*(-1);
	w3_speed = -this->x_CMD*cos(m3_Angle)+y_CMD*sin(m3_Angle)+yaw_CMD*robot_radius*(-1);
#ifdef DEBUG
	std::cout << "Inverse kinematics(DEBUG)\n";
	std::cout << std::dec;
	std::cout << "x_speed CMD: " << (base_robotCMD.x_speed) << std::endl;
	std::cout << "y_speed CMD: " << (base_robotCMD.y_speed) << std::endl;
	std::cout << "yaw_speed CMD: " << (base_robotCMD.yaw_speed) << std::endl;
	std::cout << "w1_speed(%): " << w1_speed << std::endl;
	std::cout << "w2_speed(%): " << w2_speed << std::endl;
	std::cout << "w3_speed(%): " << w3_speed << std::endl;
	std::cout << std::endl;
#endif
	speed_regularization(w1_speed, w2_speed, w3_speed);
}

void BaseControl::run()
{

}

void* BaseControl::pThreadRun(void* argv)
{
    BaseControl* Base = (BaseControl*)argv;
    Base->run();
    pthread_exit(NULL);
}

bool BaseControl::getBaseFlag()
{
//    std::cout << base_flag << std::endl;
    return base_flag;
}

serial_rx* BaseControl::getPack()
{
    base_flag = false;
    return &base_RX;
}

void BaseControl::send(const robot_command &CMD)
{
#ifdef DEBUG
	std::cout << "BaseControl::send(DEBUG)\n";
#endif
	this->base_robotCMD = CMD;
	shoot_regularization();
	inverseKinematics();
	mcssl_send2motor();
}

robot_command *BaseControl::get_feedback()
{
	forwardKinematics();
	return &base_robotFB;
}
