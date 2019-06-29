#include "motor_control.h"

uint8_t MotorController::serial_data[RX_PACKAGE_SIZE]={0};
bool MotorController::serial_flag = false;
bool MotorController::decoder_flag = false;
int MotorController::length = 0;
MotorController::MotorController()
{
    this->serial = NULL;
#ifdef _DEBUG
    std::cout << "\nMotorController(DEBUG)\n";
#endif
}

MotorController::MotorController(int argc, char** argv, bool record = false): record(record)
{
    printf("name%s\n", argv[0]);
    if(this->record){
        this->record_name = this->record_name.assign(argv[0])  + "_record.txt";
    }
	this->serial = NULL;
    this->motor_TX = {0};
    this->motor_TX.head1 = 0xff;
    this->motor_TX.head2 = 0xfa;
    this->motor_RX = {0};
    this->motor_odometry = 0;
    this->duration = 0;
    this->motor_flag = false;
    this->error_flag = false;
    this->send_flag = false;
    this->clear_odo = false;
    this->last_time = {0};
    this->crc_data[TX_PACKAGE_SIZE-2] = {0};

    this->w1_pwm = 0;
    this->w2_pwm = 0;
    this->w3_pwm = 0;
    this->pmotor_speed = NULL;
    this->curr_rpm = 0;
    this->curr_pwm = 0;
    this->tar_rpm = 0;
    this->tar_pwm = 0;
    this->pre_rpm = 0;
    this->cos_value = 0;
    this->spd_err = 0;
    this->enable = false;
    this->stop = false;


#ifdef DEBUG
    std::cout << "\nMotorController(DEBUG)\n";
//    printf("motor RX%d %d %d %d %d\n",motor_RX.id, motor_RX.duration, motor_RX.w1, motor_RX.w2, motor_RX.w3);
#endif
#ifdef CSSL
	McsslInit();
#endif
    int p = pthread_create(&tid, NULL, (THREADFUNCPTR)&MotorController::pThreadRun, this);
    if(p != 0){
        printf("motor thread error\n");
        exit(EXIT_FAILURE);
    }
}

MotorController::~MotorController()
{
    if(this->serial){
	    McsslFinish();
        this->serial = NULL;
    }
#ifdef DEBUG
	std::cout << "\n~MotorController(DEBUG)\n";
#endif
}

int MotorController::McsslInit()
{
	std::cout << "==== Init cssl ====\n";
	cssl_start();
	if(!serial){
		serial = cssl_open(this->port, McsslCallback/*NULL*/, 0, 115200, 8, 0, 1);
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

void MotorController::McsslFinish()
{
    fp.close();
    pthread_cancel(tid);
    printf("close motor thread\n");
#ifdef CSSL
	cssl_close(serial);
	cssl_stop();

    this->serial = NULL;
#else
	std::cout << "mcssl_finish(CSSL)\n";
#endif
}

void MotorController::McsslCallback(int id, uint8_t* buf, int len)
{
//    printf("duration: %f %f\n", now.tv_usec, now.tv_sec);
    serial_flag = true;
//    printf("%d\n\n", len);
    length = len;
    if(!decoder_flag){
        if((*buf == 0xff) && (*(buf + 1) == 0xfa)){
            for(int i=0; i<RX_PACKAGE_SIZE; i++){
                serial_data[i] = *(buf + i);
            }
        }

    }

}

bool MotorController::SerialDecoder()
{
    serial_flag = false;
    decoder_flag = true;
    uint8_t data[RX_PACKAGE_SIZE];
    for(int i=0; i<RX_PACKAGE_SIZE; i++){
        data[i] = serial_data[i];
    }
    decoder_flag = false;
    struct timeval now;
    gettimeofday(&now, 0);
    if((data[0] == 0xff) && (data[1] == 0xfa)){

        Crc_16 Crc16(data, sizeof(data)/sizeof(uint8_t));
        unsigned short crc_16 = Crc16.getCrc(data, sizeof(data)/sizeof(uint8_t));
//        printf("crc %d\n", crc_16);
        if(crc_16 == 0){
            long duration_s = now.tv_sec - last_time.tv_sec;
            long duration_us = now.tv_usec - last_time.tv_usec;
            last_time = now;
            motor_RX.duration = duration_s * 1000000 + duration_us;
            
            motor_RX.size = length;
            motor_RX.w1 = (data[2] << 24) + (data[3] << 16) + (data[4] << 8) + data[5];
            motor_RX.w2 = (data[6] << 24) + (data[7] << 16) + (data[8] << 8) + data[9];
            motor_RX.w3 = (data[10] << 24) + (data[11] << 16) + (data[12] << 8) + data[13];
            motor_RX.error = "no error";
            error_flag = false;
//            printf("decoer correct\n");
        }else{
            motor_RX.w1 = 0;
            motor_RX.w2 = 0;
            motor_RX.w3 = 0;
            motor_RX.error = "error";
            error_flag = true;
//            printf("decoer error\n");
        }
    }
#ifdef DEBUG_CSSLCALLBACK
    if(error_flag){
        printf("length %d \n", length);
        for(int i = 0; i < RX_PACKAGE_SIZE; i++){
            printf("%x ", data[i]);
        }
        printf("\n");

    }
#endif
    return this->error_flag;

}

void MotorController::FPGAInit()
{
}

void MotorController::McsslSend2FPGA()
{

}

void MotorController::McsslSend2FPGA(
        bool w1_en, 
        bool w2_en, 
        bool w3_en, 
        bool w1_stop, 
        bool w2_stop, 
        bool w3_stop
        ) 

{	
    this->motor_TX.w1_l = w1_pwm;
    this->motor_TX.w1_h = w1_pwm >> 8;
    this->motor_TX.w2_l = w2_pwm;
    this->motor_TX.w2_h = w2_pwm >> 8;
    this->motor_TX.w3_l = w3_pwm;
    this->motor_TX.w3_h = w3_pwm >> 8;
    this->motor_TX.enable_and_stop = (w1_en << 7) + (w2_en << 6) + (w3_en << 5) + (w1_stop << 4) + (w2_stop << 3) + (w3_stop << 2); 
    for(int i = 0; i < TX_PACKAGE_SIZE-2; i++){
        crc_data[i] = *((uint8_t*)(&motor_TX)+i);
    }

    crc_16 = Crc.getCrc(crc_data, TX_PACKAGE_SIZE -2);
//    this->motor_TX.crc_16_1 = *(unsigned char*)(&crc_16) + 1;
//    this->motor_TX.crc_16_2 = *(unsigned char*)(&crc_16) + 0;
    this->motor_TX.crc_16_1 = crc_16 >> 8;
    this->motor_TX.crc_16_2 = crc_16;
    for(int i = 0; i < TX_PACKAGE_SIZE; i++){
        cssl_data[i] = *((uint8_t*)(&motor_TX)+i);
    }
#ifdef CSSL
    cssl_putdata(serial, cssl_data, TX_PACKAGE_SIZE);
#ifdef DEBUG
    printf("**************************\n");
    printf("* mcssl_send(DEBUG) *\n");
    printf("**************************\n");
    printf("head1: %x\n", (this->motor_TX.head1));
    printf("head2: %x\n", (this->motor_TX.head2));
    printf("w1_h: %x\n", (this->motor_TX.w1_h));
    printf("w1_l: %x\n", (this->motor_TX.w1_l));
    printf("w2_h: %x\n", (this->motor_TX.w2_h));
    printf("w2_l: %x\n", (this->motor_TX.w2_l));
    printf("w3_h: %x\n", (this->motor_TX.w3_h));
    printf("w3_l: %x\n", (this->motor_TX.w3_l));
    printf("enable_and_stop: %x\n", (this->motor_TX.enable_and_stop));
    printf("shoot: %x\n", (this->motor_TX.shoot));
    printf("crc16-1: %x\n", (this->motor_TX.crc_16_1));
    printf("crc16-2: %x\n", (this->motor_TX.crc_16_2));
    printf("crc16: %x\n", (crc_16));
    printf("w1: %x\n", ((this->motor_TX.w1_h) << 8) + (this->motor_TX.w1_l));
    printf("w2: %x\n", ((this->motor_TX.w2_h) << 8) + (this->motor_TX.w2_l));
    printf("w3: %x\n", ((this->motor_TX.w3_h) << 8) + (this->motor_TX.w3_l));
#endif
#else
    printf("**************************\n");
    printf("* mcssl_send(CSSL) *\n");
    printf("**************************\n");
    printf("head1: %x\n", (this->motor_TX.head1));
    printf("head2: %x\n", (this->motor_TX.head2));
    printf("w1_h: %x\n", (this->motor_TX.w1_h));
    printf("w1_l: %x\n", (this->motor_TX.w1_l));
    printf("w2_h: %x\n", (this->motor_TX.w2_h));
    printf("w2_l: %x\n", (this->motor_TX.w2_l));
    printf("w3_h: %x\n", (this->motor_TX.w3_h));
    printf("w3_l: %x\n", (this->motor_TX.w3_l));
    printf("enable_and_stop: %x\n", (this->motor_TX.enable_and_stop));
    printf("shoot: %x\n", (this->motor_TX.shoot));
    printf("crc16-1: %x\n", (this->motor_TX.crc_16_1));
    printf("crc16-2: %x\n", (this->motor_TX.crc_16_2));
    printf("crc16: %x\n", (crc_16));
    printf("w1: %x\n", ((this->motor_TX.w1_h) << 8) + (this->motor_TX.w1_l));
    printf("w2: %x\n", ((this->motor_TX.w2_h) << 8) + (this->motor_TX.w2_l));
    printf("w3: %x\n", ((this->motor_TX.w3_h) << 8) + (this->motor_TX.w3_l));
#endif
}

void MotorController::DriverSetting(int16_t w1, int16_t w2, int16_t w3)
{
#ifdef DEBUG
    printf("DriverSetting(DEBUG)\n");
    printf("w1: %d, w2: %d, w3: %d\n", w1, w2, w3);
#endif
	bool en1 = (fabs(w1) > 0)? 1 : 0;
	bool en2 = (fabs(w2) > 0)? 1 : 0;
	bool en3 = (fabs(w3) > 0)? 1 : 0;
	bool stop1 = 0;
	bool stop2 = 0;
	bool stop3 = 0;
    this->w1_pwm = (w1>=0)? (MAX_PWM-MAX_PWM*0.2)*w1/100+MIN_PWM: (MAX_PWM-MAX_PWM*0.2)*w1/100-MIN_PWM;
    this->w2_pwm = (w2>=0)? (MAX_PWM-MAX_PWM*0.2)*w2/100+MIN_PWM: (MAX_PWM-MAX_PWM*0.2)*w2/100-MIN_PWM;
    this->w3_pwm = (w3>=0)? (MAX_PWM-MAX_PWM*0.2)*w3/100+MIN_PWM: (MAX_PWM-MAX_PWM*0.2)*w3/100-MIN_PWM;
    this->McsslSend2FPGA(
            en1,
            en2,
            en3,
            stop1,
            stop2,
            stop3
            );
}

void MotorController::SpeedControl()
{

    /***********************
    double scale = 1/(0.000001 * this->motor_RX.duration);
    this->tar_w1 = (this->tar_w1_pwm - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
    this->tar_w2 = (this->tar_w2_pwm - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
    this->tar_w3 = (this->tar_w3_pwm - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
    this->curr_w1 = this->motor_RX.w1 * scale * 60 / 2000;
    this->curr_w2 = this->motor_RX.w2 * scale * 60 / 2000;
    this->curr_w3 = this->motor_RX.w3 * scale * 60 / 2000;
    w1_spd_err = this->CalSpdErr(tar_w1, curr_w1);
    w2_spd_err = this->CalSpdErr(tar_w2, curr_w2);
    w3_spd_err = this->CalSpdErr(tar_w3, curr_w3);
    if(CalSpdErr((double)tar_w1_pwm, (double)w1_pwm) >= 1){
        w1_cos_value += 0.02;
        std::min(M_PI/2, w1_cos_value);
        this->w1_pwm = MAX_PWM * sin(w1_cos_value);
    }else{
        w1_cos_value -= 0.02;
        std::max(-M_PI/2, w1_cos_value);
        this->w1_pwm = MAX_PWM * sin(w1_cos_value);

    }
    if(w2_spd_err>=0){
        w2_cos_value += 0.02;
        std::min(M_PI/2, w2_cos_value);
        this->w2_pwm = MAX_PWM * sin(w2_cos_value);
    }else{
        w2_cos_value -= 0.02;
        std::max(-M_PI/2, w2_cos_value);
        this->w2_pwm = MAX_PWM * sin(w2_cos_value);

    }
    if(w3_spd_err>=0){
        w3_cos_value += 0.02;
        std::min(M_PI/2, w3_cos_value);
        this->w3_pwm = MAX_PWM * sin(w3_cos_value);
    }else{
        w3_cos_value -= 0.02;
        std::max(-M_PI/2, w3_cos_value);
        this->w3_pwm = MAX_PWM * sin(w3_cos_value);

    }
    ************************/
}

double MotorController::PWM2RPM(int16_t pwm)
{
    return (pwm - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
}

int16_t MotorController::RPM2PWM(double rpm)
{
    return rpm * (MAX_PWM - MAX_PWM * 0.2) / MAX_MOTOR_RPM + MIN_PWM;
}

double MotorController::SpdErr(double target, double current)    //calculate the motor speed error
{
    return target - current;

}

void MotorController::SpeedRegularization(double w1_p, double w2_p, double w3_p)
{

}

void MotorController::Run()
{
    std::stringstream fss;
    std::stringstream fduration;
    if(record){
        fp.open(this->record_name, std::ios::out);
        if(fp.is_open())
            printf("open record file\n");
        else
            printf("file open failed\n");

    }else{
        printf("do not record file\n");
    }
//    printf("in the run\n");
    gettimeofday(&last_time, 0);
    double scale = 0;
    while(true){
        if(this->serial_flag){
            this->motor_flag = true;
            if(this->SerialDecoder()){
#ifdef DEBUG_CSSLCALLBACK
                printf("serial decode fail\n");
#endif
            }else{
                scale =  1/(0.000001 * motor_RX.duration) * 60;
                this->curr_rpm = (*pmotor_speed) * scale / 2000;
                if(this->clear_odo){
                    this->clear_odo = false;
                    motor_odometry = 0;
                    duration = 0;
                }
                if(pmotor_speed){
                    motor_odometry += *pmotor_speed;
                    this->duration += this->motor_RX.duration;
                }
#ifdef _RECORD  // record 
                if(record && (fabs(this->curr_rpm) < MAX_MOTOR_RPM) && pmotor_speed){

//                    printf("scale%d %f\n", motor_RX.duration,1/(0.000001*motor_RX.duration));
                    if(tar_pwm>0){
                        tar_rpm = (tar_pwm - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
                    }else if(tar_pwm<0){
                        tar_rpm = -(fabs(tar_pwm) - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
                    }else{
                        tar_rpm = 0;
                    }

                    fss << this->motor_RX.duration << " "
                        << this->curr_rpm << " "
                        << this->tar_rpm  << " " 
                        << std::endl;
                    fp << fss.str();
                    fss.str(std::string());


                }
#endif

            }
        }else{
//            printf("cannot get serial\n");

        }
    }
}

void* MotorController::pThreadRun(void* p)
{
    ((MotorController*)p)->Run();
    pthread_exit(NULL);
}

bool MotorController::GetMotorFlag()
{
    if(motor_flag){
        motor_flag = false;
        return true;

    }else
        return false;
}

bool MotorController::GetErrFlag()
{
    return error_flag;
}

uint8_t* MotorController::GetPacket()
{
    return serial_data;
}

double MotorController::GetSpeed()
{
    this->clear_odo = true;
    double scale = 1 / (0.000001 * this->duration);
    return motor_odometry*scale*60/2000;
}

long MotorController::GetDuration()
{
    return duration;
}

void MotorController::SetSpeed(int number, double p)
{
    if(p>0)this->tar_pwm = p*(MAX_PWM-0.2*MAX_PWM)/100+MIN_PWM;
    else if(p<0)this->tar_pwm = p*(MAX_PWM-0.2*MAX_PWM)/100-MIN_PWM;
    else this->tar_pwm = 0;

    switch(number){
        case 1:
            pmotor_speed = &(this->motor_RX.w1);
            DriverSetting(p, 0, 0);
            break;
        case 2:
            pmotor_speed = &(this->motor_RX.w2);
            DriverSetting(0, p, 0);
            break;
        case 3:
            pmotor_speed = &(this->motor_RX.w3);
            DriverSetting(0, 0, p);
            break;
        default:
            DriverSetting(0, 0, 0);
            break;
    }
    this->motor_TX.shoot = 0;
    this->send_flag = true;
}

