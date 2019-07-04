#include "motor_control.h"

bool MotorController::serial_flag = false;
bool MotorController::decoder_flag = false;
int MotorController::length = 0;
SerialData MotorController::serialData;
MotorController::MotorController(
        int argc, 
        char **argv, 
        bool record = false, 
        int number = 1, 
        std::string name = "motor1"): 
    record(record), m_number(number), m_name(name)
{
    serialData.data[BUFFER_SIZE] = {0};
    serialData.head = 0;
    serialData.rear = 0;
//    printf("name%s\n", argv[0]);
    this->pmotor_pwm = new int;
    std::memset(this->pmotor_pwm, 0, sizeof(int));
    this->pmotor_feedback = new int;
    std::memset(this->pmotor_feedback, 0, sizeof(int));
    this->motor_RX = {0};
    this->motorPWM = {0};
    switch(m_number){
        case 1:
            pmotor_feedback = &(this->motor_RX.w1);
            pmotor_pwm = &(this->motorPWM.w1);
            break;
        case 2:
            pmotor_feedback = &(this->motor_RX.w2);
            pmotor_pwm = &(this->motorPWM.w2);
            break;
        case 3:
            pmotor_feedback = &(this->motor_RX.w3);
            pmotor_pwm = &(this->motorPWM.w3);
            break;
        default:
            pmotor_feedback = &(this->motor_RX.w1);
            pmotor_pwm = &(this->motorPWM.w1);
            break;
    }
    if(this->record){
        this->record_name = this->record_name.assign(argv[0])  + "_record.txt";
    }
	this->serial = NULL;
    this->motor_TX = {0};
    this->motor_TX.head1 = 0xff;
    this->motor_TX.head2 = 0xfa;
    this->motor_odometry = 0;
    this->duration = 0;
    this->motor_flag = false;
    this->decoder_error = false;
//    this->bsend = false;
    this->benable = false;
    this->clear_odo = false;
    this->close = false;
    this->last_time = {0};
    this->curr_rpm = 0;
    this->curr_pwm = 0;
    this->tar_rpm = 0;
    this->tar_pwm = 0;
    this->sin_value = 0;
    this->error_rpm = 0;
    this->pre_error_rpm = 0;
//    this->delta_error_rpm = 0;
    this->motor_enable = false;
    this->motor_stop = false;
#ifdef DEBUG
    std::cout << "\nMotorController(DEBUG)\n";
#endif //DEBUG
#ifdef CSSL
	McsslInit();
#endif //CSLL
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
    if(fp.is_open())
        fp.close();
//    pthread_cancel(tid);
#ifdef DEBUG
    printf("close motor thread\n");
	std::cout << "\n~MotorController(DEBUG)\n";
#endif //DEBUG
}

int MotorController::McsslInit()
{
	std::cout << "==== Init cssl ====\n";
	cssl_start();
	if(!serial){
		serial = cssl_open("/dev/communication/motion", McsslCallback/*NULL*/, 0, 115200, 8, 0, 1);
//		serial = cssl_open(this->port.c_str(), NULL, 0, 115200, 8, 0, 1);
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

#ifdef CSSL
	cssl_close(serial);
	cssl_stop();

    this->serial = NULL;
#else //no CSSL
	std::cout << "mcssl_finish(no CSSL)\n";
#endif //CSSL
}

void MotorController::McsslCallback(int id, uint8_t* buf, int len)
{
    serial_flag = true;
    length = len;
//    if(!decoder_flag){
        for(int i=0; i<len; i++){
            serialData.rear = (serialData.rear+1)%BUFFER_SIZE;
            if(serialData.rear >= BUFFER_SIZE)serialData.rear = 0;
            serialData.data[serialData.rear] = *(buf + i);
//            serialData.rear = serialData.rear % BUFFER_SIZE;
        }

//    }

}

bool MotorController::SerialDecoder()
{
#ifndef CSSL
    for(int i=0; i<17; i++){
        serialData.rear++;
        if(serialData.rear >= BUFFER_SIZE)serialData.rear = 0;
        if(i==0)serialData.data[serialData.rear++] = 0xff;
        else if(i==1)serialData.data[serialData.rear++] = 0xfa;
        else if(i==14)serialData.data[serialData.rear++] = 0xe9;
        else if(i==15)serialData.data[serialData.rear++] = 0xa4;
        else serialData.data[serialData.rear++] = 0;

        if(serialData.rear>=BUFFER_SIZE)serialData.rear=0;
    }
#ifdef DEBUG_CSSLCALLBACK
    printf("*****Buffer data*****\n");
    printf("serialData.head: %d, serialData.rear: %d data length%d", serialData.head, serialData.rear,dat_len);
    for(int i = 0; i < BUFFER_SIZE; i++){
        if(i%10==0)printf("\n");
        printf("%x ", serialData.data[i]);
    }
    printf("\n");
#endif //DEBUG_CSSLCALLBACK
#endif //CSSL
//    serial_flag = false;
    int data_len = serialData.rear - serialData.head;
//    printf("data lengh%d serialData.head%d serialData.rear%d decode_flag%d, serial_flag%d\n", data_len, serialData.head, serialData.rear, decoder_flag, serial_flag);
    serial_flag = false;
    if(data_len == 0)return false;
    if(data_len < 0)data_len = data_len + BUFFER_SIZE;
    if(data_len < RX_PACKAGE_SIZE) return false;
    decoder_flag = true;
    for(int i = 0; i <= (data_len-RX_PACKAGE_SIZE); i++){
        serialData.head = (serialData.head + 1) % BUFFER_SIZE;

        if((serialData.data[serialData.head]==0xff)&&(serialData.data[(serialData.head+1)%BUFFER_SIZE]==0xfa)){
            for(int j = 0; j < RX_PACKAGE_SIZE; j++){
                this->serial_data[j] = serialData.data[(serialData.head+j)%BUFFER_SIZE];
            }
            crc_16 = Crc.getCrc(serial_data, RX_PACKAGE_SIZE);
            if(crc_16 == 0){
                gettimeofday(&now, 0);
                long duration_s = now.tv_sec - last_time.tv_sec;
                long duration_us = now.tv_usec - last_time.tv_usec;
                last_time = now;
                motor_RX.duration = duration_s * 1000000 + duration_us;

                motor_RX.size = length;
                motor_RX.w1 = (serial_data[2] << 24) + (serial_data[3] << 16) + (serial_data[4] << 8) + serial_data[5];
                motor_RX.w2 = (serial_data[6] << 24) + (serial_data[7] << 16) + (serial_data[8] << 8) + serial_data[9];
                motor_RX.w3 = (serial_data[10] << 24) + (serial_data[11] << 16) + (serial_data[12] << 8) + serial_data[13];
                motor_RX.error = "no error";
                decoder_error = false;
                bdecoder_success = true;
                serialData.head = (serialData.head + RX_PACKAGE_SIZE)%BUFFER_SIZE;
//                printf("decoer correct\n");
                break;
            }else{
//                motor_RX.w1 = 0;
//                motor_RX.w2 = 0;
//                motor_RX.w3 = 0;
                motor_RX.error = "error";
                decoder_error = true;
                bdecoder_success = false;
//                printf("decoer error\n");
            }
        }
        decoder_error = true;
        this->bdecoder_success = false;
    }
//    printf("speed%d speed%d, duration: %d\n", *pmotor_feedback, motor_RX.w1, (int)motor_RX.duration);
#ifdef DEBUG_CSSLCALLBACK
    printf("Buffer data\n");
    printf("serialData.head: %d, serialData.rear: %d, duration: %d\n", serialData.head, serialData.rear, (int)motor_RX.duration);
    printf("\nserial data:\tlength %d \n", length);
    for(int i = 0; i < RX_PACKAGE_SIZE; i++){
        printf("%x ", serial_data[i]);
    }
    printf("\n\n");

#endif //DEBUG_CSSLCALLBACK
    return this->bdecoder_success;

}

void MotorController::FPGAInit()
{
}

void MotorController::McsslSend2FPGA()
{
    this->motor_TX.w1_l = motorPWM.w1;
    this->motor_TX.w1_h = motorPWM.w1 >> 8;
    this->motor_TX.w2_l = motorPWM.w2;
    this->motor_TX.w2_h = motorPWM.w2 >> 8;
    this->motor_TX.w3_l = motorPWM.w3;
    this->motor_TX.w3_h = motorPWM.w3 >> 8;
    switch(m_number){
        case 1:
            this->motor_TX.enable_and_stop = (this->motor_enable << 7) + (this->motor_stop << 4);

            break;
        case 2:
            this->motor_TX.enable_and_stop = (this->motor_enable << 6) + (this->motor_stop << 3);
            break;
        case 3:
            this->motor_TX.enable_and_stop = (this->motor_enable << 5) + (this->motor_stop << 2);
            break;
        default:
            this->motor_TX.enable_and_stop = this->motor_enable << 7 + this->motor_stop << 4;
            break;
    }
//    this->motor_TX.enable_and_stop = (w1_en << 7) + (w2_en << 6) + (w3_en << 5) + (w1_stop << 4) + (w2_stop << 3) + (w3_stop << 2); 
    for(int i = 0; i < TX_PACKAGE_SIZE-2; i++){
        cssl_data[i] = *((uint8_t*)(&motor_TX)+i);
    }

    crc_16 = Crc.getCrc(cssl_data, TX_PACKAGE_SIZE -2);
//    this->motor_TX.crc_16_1 = *(unsigned char*)(&crc_16) + 1;
//    this->motor_TX.crc_16_2 = *(unsigned char*)(&crc_16) + 0;
    this->motor_TX.crc_16_1 = crc_16 >> 8;
    this->motor_TX.crc_16_2 = crc_16;
    cssl_data[TX_PACKAGE_SIZE-2] = this->motor_TX.crc_16_1;
    cssl_data[TX_PACKAGE_SIZE-1] = this->motor_TX.crc_16_2;
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
#endif //DEBUG
#else // no CSSL
    printf("**************************\n");
    printf("* mcssl_send(no CSSL) *\n");
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
#endif //CSSL

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
    this->motor_TX.w1_l = motorPWM.w1;
    this->motor_TX.w1_h = motorPWM.w1 >> 8;
    this->motor_TX.w2_l = motorPWM.w2;
    this->motor_TX.w2_h = motorPWM.w2 >> 8;
    this->motor_TX.w3_l = motorPWM.w3;
    this->motor_TX.w3_h = motorPWM.w3 >> 8;
    this->motor_TX.enable_and_stop = (w1_en << 7) + (w2_en << 6) + (w3_en << 5) + (w1_stop << 4) + (w2_stop << 3) + (w3_stop << 2); 
    for(int i = 0; i < TX_PACKAGE_SIZE-2; i++){
        cssl_data[i] = *((uint8_t*)(&motor_TX)+i);
    }

    crc_16 = Crc.getCrc(cssl_data, TX_PACKAGE_SIZE -2);
//    this->motor_TX.crc_16_1 = *(unsigned char*)(&crc_16) + 1;
//    this->motor_TX.crc_16_2 = *(unsigned char*)(&crc_16) + 0;
    this->motor_TX.crc_16_1 = crc_16 >> 8;
    this->motor_TX.crc_16_2 = crc_16;
    cssl_data[TX_PACKAGE_SIZE-2] = this->motor_TX.crc_16_1;
    cssl_data[TX_PACKAGE_SIZE-1] = this->motor_TX.crc_16_2;
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
#endif //DEBUG
#else // no CSSL
    printf("**************************\n");
    printf("* mcssl_send(no CSSL) *\n");
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
#endif //CSSL
}

void MotorController::DriverSetting()
{
#ifdef DEBUG
    printf("DriverSetting(DEBUG)\n");
//    printf("w1: %d, w2: %d, w3: %d\n", motorCommand.w1, motorCommand.w2, motorCommand.w3);
#endif //DEBUG
    if(this->close){
        this->motor_enable = 0;
        this->motor_stop = 0;
        *this->pmotor_pwm = 0;
    }else{
        this->motor_enable = (fabs(this->curr_pwm)>=MIN_PWM)? 1 : 0;
        this->motor_stop = (fabs(this->curr_pwm)>MIN_PWM)? 0 : 1;
        *this->pmotor_pwm = this->curr_pwm;
    }
#ifndef CSSL
    serial_flag = true;
#endif //CSSL

}

void MotorController::DriverSetting(int16_t w1, int16_t w2, int16_t w3)
{
#ifdef DEBUG
    printf("DriverSetting(DEBUG)\n");
    printf("w1: %d, w2: %d, w3: %d\n", w1, w2, w3);
#endif //DEBUG
	bool en1 = (fabs(w1) > 0)? 1 : 0;
	bool en2 = (fabs(w2) > 0)? 1 : 0;
	bool en3 = (fabs(w3) > 0)? 1 : 0;
	bool stop1 = 0;
	bool stop2 = 0;
	bool stop3 = 0;
    this->motorPWM.w1 = (w1>=0)? (MAX_PWM-MAX_PWM*0.2)*w1/100+MIN_PWM: (MAX_PWM-MAX_PWM*0.2)*w1/100-MIN_PWM;
    this->motorPWM.w2 = (w2>=0)? (MAX_PWM-MAX_PWM*0.2)*w2/100+MIN_PWM: (MAX_PWM-MAX_PWM*0.2)*w2/100-MIN_PWM;
    this->motorPWM.w3 = (w3>=0)? (MAX_PWM-MAX_PWM*0.2)*w3/100+MIN_PWM: (MAX_PWM-MAX_PWM*0.2)*w3/100-MIN_PWM;
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
    error_rpm = tar_rpm - curr_rpm;
    double tar_sin_value = asin(tar_rpm/MAX_MOTOR_RPM);
//    printf("%f\n", sin_value);
//    double error = tar_sin_value - sin_value;
//    double delta_error = error - pre_error_rpm;
//    double acc_error = error + pre_error_rpm;
//    double delta_error_rpm = error_rpm - pre_error_rpm;
//    double acc_error_rpm = error_rpm + pre_error_rpm;
    double k_p = 0.015;
    double k_i = 0 ;
    double k_d = 0;
    curr_pwm = RPM2PWM(sin(sin_value)*max_rpm);

//    sin_value = (k_p * error_rpm + k_i * acc_error_rpm + k_d * delta_error_rpm);
//    double delta_sin_value = (k_p *error + k_i * acc_error + k_d * delta_error);
    if(curr_rpm >= 0){
        if(error_rpm > 0 /*&& (fabs(error_rpm)>100)*/){
//            sin_value += (0.02 + acc_error_rpm);
            sin_value += (0.03);
            if(sin_value >= M_PI/2)sin_value = M_PI/2;
        }
        if(error_rpm < 0 /*&& (fabs(error_rpm)>100)*/){
//            sin_value += ( + acc_error_rpm);
            sin_value -= (0.02);
            if(sin_value <= -M_PI/2)sin_value = -M_PI/2;
        }

    }else if(curr_rpm < 0){
        if(error_rpm > 0 /*&& (fabs(error_rpm)>100)*/){
//            sin_value -= (0.01 + acc_error_rpm);
            sin_value += (0.02);
            if(sin_value >= M_PI/2)sin_value = M_PI/2;
        }
        if(error_rpm < 0 /*&& (fabs(error_rpm)>100)*/){
//            sin_value -= (0.02 + acc_error_rpm);
            sin_value -= (0.03);
            if(sin_value <= -M_PI/2)sin_value = -M_PI/2;
        }

    }
    if(fabs(curr_rpm)<=MIN_MOTOR_RPM && (tar_rpm * sin_value <=0)){
        sin_value = 0;
    }
//    pre_error_rpm = error_rpm;
    pre_error_rpm = sin_value;
//    printf("%f\n",sin_value);

}

double MotorController::GetSinValue()
{
    return asin(this->curr_rpm/fabs(this->max_rpm));
}

double MotorController::PWM2RPM(int16_t pwm)
{
    if(pwm > 0) 
        return (pwm - MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
    else if(pwm < 0)
        return (pwm + MIN_PWM) * MAX_MOTOR_RPM / (MAX_PWM - MAX_PWM * 0.2);
    else 
        return 0;
}

int16_t MotorController::RPM2PWM(double rpm)
{
    if(rpm > 0) 
        return rpm * (MAX_PWM-MAX_PWM*0.2)/MAX_MOTOR_RPM + MIN_PWM;
    else if(rpm < 0)
        return rpm * (MAX_PWM-MAX_PWM*0.2)/MAX_MOTOR_RPM - MIN_PWM;
    else 
        return 0;
}

double MotorController::SpdErr(double target, double current)    //calculate the motor speed error
{
    return target - current;

}

bool MotorController::GetMotorFlag()
{
    if(motor_flag){
        motor_flag = false;
        return true;

    }else
        return false;
}

bool MotorController::GetDecoderErrFlag()
{
    return decoder_error;
}

uint8_t* MotorController::GetPacket()
{
//    return serial_data;
    return serialData.data;
}

double MotorController::GetTarRPM()
{
    return this->tar_rpm;
}

double MotorController::GetCurrRPM()
{
    return this->curr_rpm;
}

double MotorController::GetOdometry()
{
    this->clear_odo = true;
    double scale = FB_FREQUENCY * 60;
    return motor_odometry*scale/2000;
}

int16_t MotorController::GetTarPWM()
{
    return this->tar_pwm;
}

int16_t MotorController::GetCurrPWM()
{
    return this->curr_pwm;
}

long MotorController::GetDuration()
{
    return duration;
}

void MotorController::SetMotor(int number, std::string name)
{
    this->m_name = name;
    this->m_number = number;
    switch(m_number){
        case 1:
            pmotor_feedback = &(this->motor_RX.w1);
            pmotor_pwm = &(this->motorPWM.w1);
            break;
        case 2:
            pmotor_feedback = &(this->motor_RX.w2);
            pmotor_pwm = &(this->motorPWM.w2);
            break;
        case 3:
            pmotor_feedback = &(this->motor_RX.w3);
            pmotor_pwm = &(this->motorPWM.w3);
            break;
        default:
            pmotor_feedback = &(this->motor_RX.w1);
            pmotor_pwm = &(this->motorPWM.w1);
            break;
    }
}

void MotorController::SetSpeed(double p)
{
    this->tar_rpm = p/100*MAX_MOTOR_RPM;
//    if(fabs(tar_rpm) < PWM2RPM(MIN_PWM+1))this->tar_rpm = 0;
    this->tar_pwm = this->RPM2PWM(this->tar_rpm);
//    this->sin_value = this->GetSinValue();
    this->max_rpm = fabs(this->tar_rpm);

    switch(m_number){
        case 1:
            this->motorPWM.w1 = this->tar_pwm;
            this->motorPWM.w2 = 0;
            this->motorPWM.w3 = 0;
            break;
        case 2:
            this->motorPWM.w1 = 0;
            this->motorPWM.w2 = this->tar_pwm;
            this->motorPWM.w3 = 0;;
            break;
        case 3:
            this->motorPWM.w1 = 0;
            this->motorPWM.w2 = 0;
            this->motorPWM.w3 = this->tar_pwm;
            break;
        default:
            this->motorPWM.w1 = 0;
            this->motorPWM.w2 = 0;
            this->motorPWM.w3 = 0;
            break;
    }
    this->motor_TX.shoot = 0;
}

void MotorController::SetTarRPM(int rpm)
{
    this->SetSpeed(rpm/MAX_MOTOR_RPM);
}

void MotorController::SetEnable()
{
    this->benable = true;
}

void MotorController::ClearOdo()
{
    this->clear_odo = true;
}

void MotorController::Close()
{
    this->close = true;
    this->DriverSetting();
    this->McsslSend2FPGA();
}

void* MotorController::pThreadRun(void* p)
{
    ((MotorController*)p)->Run();
    pthread_exit(NULL);
}

void MotorController::Run()
{
    std::stringstream fss;
    std::stringstream fduration;
#ifdef RECORD
    if(record){
        fp.open(this->record_name, std::ios::out);
        if(fp.is_open())
            printf("open record file\n");
        else
            printf("file open failed\n");

    }else{
        printf("do not record file\n");
    }
#endif
//    printf("in the run\n");
    gettimeofday(&last_time, 0);
    double scale = 0;
    int count=0;
    while(true){
        if(this->benable){
            this->benable = false;
            this->SpeedControl();
            this->DriverSetting();
            this->McsslSend2FPGA();
        }
        if(this->serial_flag){
//            this->SpeedControl();
            this->motor_flag = true;
            if(this->SerialDecoder()){
                scale =  FB_FREQUENCY * 60;
                this->curr_rpm = (*pmotor_feedback) * scale / 2000;
                if(this->clear_odo){
                    this->clear_odo = false;
                    motor_odometry = 0;
                    duration = 0;
                    count = 0;
                }
                this->motor_odometry += this->curr_rpm;
                this->duration = this->motor_RX.duration;
//                printf("odo %f avg %f\n", motor_odometry, avg_rpm);
#ifdef RECORD  // record 
                if(record /*&& (fabs(this->curr_rpm) < MAX_MOTOR_RPM)*/ && pmotor_feedback){
                    fss << this->motor_RX.duration << " "
                        << RPM2PWM(this->curr_rpm) << " "
                        << RPM2PWM(this->tar_rpm)  << " " 
                        << this->curr_rpm << " "
                        << this->tar_rpm  << " " 
                        << this->error_rpm  << " " 
                        << std::endl;
                    fp << fss.str();
                    fss.str(std::string());
                }
#endif //RECORD

#ifdef DEBUG_CSSLCALLBACK
                printf("decoder success\n");
#endif //DEBUD_CSSLCALLBACK
            }else{
#ifdef DEBUG_CSSLCALLBACK_
                printf("decoder fail\n");
#endif //DEBUD_CSSLCALLBACK
            }
    decoder_flag = true;
        }else{
#ifdef DEBUG_CSSLCALLBACK_
            printf("cannot get serial\n");
#endif //DEBUD_CSSLCALLBACK_

        }
    }
}

