#include "base_control.h"

SerialData BaseController::ms_serialData;
bool BaseController::msb_serial;
//bool BaseController::msb_decoder= false;
int BaseController::ms_length;

BaseController::BaseController(int argc, char** argv, bool record = false):
    mb_record(record)
    
{
    msb_serial = false;
    ms_length = 0;
    ms_serialData.head = 0;
    ms_serialData.rear = 0;

    if(mb_record){
        record_name = record_name.assign(argv[0])  + ".txt";
    }

	serial = NULL;
    m_motorCommand = {0};
    m_motorCurrPWM = {0};
    m_motorTarPWM = {0};
    m_motorCurrRPM = {0};
    m_motorTarRPM = {0};
	m_baseCommand = {0};
	m_baseOdometry = {0};
	m_baseTX = {0};
    m_baseTX.head1 = 0xff;
    m_baseTX.head2 = 0xfa;
	m_baseRX = {0};
    mb_success = false;
    mb_clear_odo = false;
    mb_base = false;
    mb_enable = false;
    mb_stop = false;
    mb_close = false;
    m_duration = 0;
    m_serialNow = {0};
    m_serialLast = {0};
    m_commandTime = {0};

    m_max_speed = 0;
    m_en_stop = 0;
    m_shoot_power = 0;
    mb_hold_ball = false;
    mb_remote = false;

    cssl_data[TX_PACKAGE_SIZE] = {0};
    serial_data[RX_PACKAGE_SIZE] = {0};
    crc_16 = 0;
    int p = pthread_create(&tid, NULL, (THREADFUNCPTR)&BaseController::mpThreadRun, this);
    if(p != 0){
        printf("base thread error\n");
        exit(EXIT_FAILURE);
    }
#ifdef CSSL
	mCsslInit();
#endif
#ifdef DEBUG
    std::cout << "\nBaseController(DEBUG)\n";
#endif
}

BaseController::~BaseController()
{
    if(serial){
	    mCsslFinish();
        serial = NULL;
    }
    mCloseRecordFile();
    pthread_cancel(tid);
    printf("close base thread\n");
#ifdef DEBUG
	std::cout << "\n~BaseController(DEBUG)\n";
#endif
}

void* BaseController::mpThreadRun(void* p)
{
    ((BaseController*)p)->mRun();
    pthread_exit(NULL);
}

void BaseController::mRun()
{
// TODO
    std::stringstream fss;
    std::stringstream fduration;
    if(mb_record){
        mOpenRecordFile();
    }
    gettimeofday(&m_serialLast, 0);
    while(true){
        if(!mCheckSerial()){
            printf("Cannot get feedback\n");
            sleep(1);
            continue;
        }
        if(mb_enable){
            mb_enable = false;
            mBaseControl();
            mDriverSetting();
            mCsslSend2FPGA();
        }
        if(msb_serial){
            mb_base = true;
            if(mSerialDecoder()){
                mSpeedRegularization();
                m_duration += m_baseRX.duration;
//                if(mb_clear_odo){
//                    mb_clear_odo = false;

#ifdef RECORD  // record TODO
                if(mb_record){
                    // motor1
                    fss << m_baseRX.duration << " " 
                        << m_motorCurrRPM.w1 << " "
                        << m_motorCurrRPM.w2 << " "
                        << m_motorCurrRPM.w3 << " "
                        << m_motorTarRPM.w1 << " "
                        << m_motorTarRPM.w2 << " "
                        << m_motorTarRPM.w3 << " "
                        << "\n";
                    fp << fss.str();
                    fss.str(std::string());
#endif
//
                }
//
            }
        }
    }
}

int BaseController::mCsslInit()
{
	std::cout << "==== Init cssl ====\n";
	cssl_start();
	if(!serial){
		serial = cssl_open("/dev/communication/motion", mCsslCallback/*NULL*/, 0, 115200, 8, 0, 1);
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

void BaseController::mCsslFinish()
{
#ifdef CSSL
	cssl_close(serial);
	cssl_stop();

    serial = NULL;
#else
	std::cout << "mcssl_finish(CSSL)\n";
#endif
}

void BaseController::mCsslSend2FPGA()
{	
    m_baseTX.w1_l = (int16_t)m_motorCurrPWM.w1;
    m_baseTX.w1_h = (int16_t)m_motorCurrPWM.w1 >> 8;
    m_baseTX.w2_l = (int16_t)m_motorCurrPWM.w2;
    m_baseTX.w2_h = (int16_t)m_motorCurrPWM.w2 >> 8;
    m_baseTX.w3_l = (int16_t)m_motorCurrPWM.w3;
    m_baseTX.w3_h = (int16_t)m_motorCurrPWM.w3 >> 8;
// TODO
    m_baseTX.enable_stop = m_en_stop;
    for(int i = 0; i < TX_PACKAGE_SIZE-2; i++){
        cssl_data[i] = *((uint8_t*)(&m_baseTX)+i);
    }

    crc_16 = Crc.getCrc(cssl_data, TX_PACKAGE_SIZE -2);
//    m_baseTX.crc_16_1 = *(unsigned char*)(&crc_16) + 1;
//    m_baseTX.crc_16_2 = *(unsigned char*)(&crc_16) + 0;
    m_baseTX.crc_16_1 = crc_16 >> 8;
    m_baseTX.crc_16_2 = crc_16;
    cssl_data[TX_PACKAGE_SIZE-2] = m_baseTX.crc_16_1;
    cssl_data[TX_PACKAGE_SIZE-1] = m_baseTX.crc_16_2;
#ifdef CSSL
    cssl_putdata(serial, cssl_data, TX_PACKAGE_SIZE);
#else
    msb_serial = true;
#endif //CSSL
#ifdef DEBUG
    ShowCsslSend();
#endif //DEBUG

}

void BaseController::mCsslCallback(int id, uint8_t* buf, int len)
{
    msb_serial = true;
    ms_length = len;
    for(int i=0; i<len; i++){
        ms_serialData.rear = (ms_serialData.rear+1)%BUFFER_SIZE;
        if(ms_serialData.rear >= BUFFER_SIZE)ms_serialData.rear = 0;
        ms_serialData.data[ms_serialData.rear] = *(buf + i);
    }
}

bool BaseController::mCheckSerial()
{
#ifndef CSSL
    return true;
#endif
    gettimeofday(&m_serialNow, 0);
    long duration_s = m_serialNow.tv_sec - m_serialLast.tv_sec;
    long duration_us = m_serialNow.tv_usec - m_serialLast.tv_usec;
    double duration = duration_s * 1000 + duration_us / 1000;
    if(duration > 1000.0){
        m_serialLast = m_serialNow;
        return false;
    }else{
        return true;
    }
}

bool BaseController::mSerialDecoder()
{
#ifndef CSSL
    gettimeofday(&m_serialNow, 0);
    long duration_s = m_serialNow.tv_sec - m_serialLast.tv_sec;
    long duration_us = m_serialNow.tv_usec - m_serialLast.tv_usec;
    m_serialLast = m_serialNow;
    m_baseRX.duration = duration_s * 1000000 + duration_us;

    m_baseRX.size = 0;
    m_baseRX.w1 = 0;
    m_baseRX.w2 = 0;
    m_baseRX.w3 = 0; 
    m_baseRX.error = "no error";
    return true;
#endif //CSSL
    msb_serial = false;
    int data_len = ms_serialData.rear - ms_serialData.head;
    if(data_len == 0)return false;
    if(data_len < 0)data_len = data_len + BUFFER_SIZE;
    if(data_len < RX_PACKAGE_SIZE) return false;
//    decoder_flag = true;
    for(int i = 0; i <= (data_len-RX_PACKAGE_SIZE); i++){
        ms_serialData.head = (ms_serialData.head + 1) % BUFFER_SIZE;

        if((ms_serialData.data[ms_serialData.head]==0xff)&&(ms_serialData.data[(ms_serialData.head+1)%BUFFER_SIZE]==0xfa)){
            for(int j = 0; j < RX_PACKAGE_SIZE; j++){
                serial_data[j] = ms_serialData.data[(ms_serialData.head+j)%BUFFER_SIZE];
            }
            crc_16 = Crc.getCrc(serial_data, RX_PACKAGE_SIZE);
            if(crc_16 == 0){
                gettimeofday(&m_serialNow, 0);
                long duration_s = m_serialNow.tv_sec - m_serialLast.tv_sec;
                long duration_us = m_serialNow.tv_usec - m_serialLast.tv_usec;
                m_serialLast = m_serialNow;
                m_baseRX.duration = duration_s * 1000000 + duration_us;

                m_baseRX.size = ms_length;
                m_baseRX.w1 = (serial_data[2] << 24) + (serial_data[3] << 16) + (serial_data[4] << 8) + serial_data[5];
                m_baseRX.w2 = (serial_data[6] << 24) + (serial_data[7] << 16) + (serial_data[8] << 8) + serial_data[9];
                m_baseRX.w3 = (serial_data[10] << 24) + (serial_data[11] << 16) + (serial_data[12] << 8) + serial_data[13];
                m_baseRX.error = "no error";
                mb_success = true;
                ms_serialData.head = (ms_serialData.head + RX_PACKAGE_SIZE)%BUFFER_SIZE;
                break;
            }else{
                m_baseRX.error = "error";
                mb_success = false;
                m_baseRX.error_times++;
            }
        }
        mb_success = false;
    }
    return mb_success;

}

void BaseController::mOpenRecordFile()
{
    if(mb_record){
        fp.open(record_name, std::ios::out);
        if(fp.is_open())
            printf("open record file1\n");
        else
            printf("file1 open failed\n");
    }else{
        printf("do not record file\n");
    }

}

void BaseController::mCloseRecordFile()
{
    if(fp.is_open())
        fp.close();
}

void BaseController::mCommandRegularization()
{
    double speed = sqrt(pow(m_baseCommand.x, 2) + pow(m_baseCommand.y, 2)) + fabs(m_baseCommand.yaw);
    if(speed > 100){

        m_baseCommand.x = m_baseCommand.x * 100 / speed;
        m_baseCommand.y = m_baseCommand.y * 100 / speed;
        m_baseCommand.yaw = m_baseCommand.yaw * 100 / speed;
        m_max_speed = 100;
    }else{
        m_max_speed = speed;
    }
    m_Motor1.SetMaxRPM(m_max_speed);
    m_Motor2.SetMaxRPM(m_max_speed);
    m_Motor3.SetMaxRPM(m_max_speed);
}

void BaseController::mSpeedRegularization()
{
    double scale = FB_FREQUENCY * 60;
    m_motorCurrRPM.w1 = m_baseRX.w1 * scale / TICKS_PER_ROUND;
    m_motorCurrRPM.w2 = m_baseRX.w2 * scale / TICKS_PER_ROUND;
    m_motorCurrRPM.w3 = m_baseRX.w3 * scale / TICKS_PER_ROUND;
}

int16_t BaseController::mPWMRegularization(int16_t pwm)
{
    if(pwm > 0){
        pwm = pwm * 0.8 + MIN_PWM;
    }else if(pwm < 0){
        pwm = pwm * 0.8 - MIN_PWM;

    }else{
        pwm = 0;
    }
    return pwm;
}

void BaseController::mShootRegularization()
{
	if(m_shoot_power == 0){
		m_shoot_power = 0;
	}else if(m_shoot_power >= 100){
		m_shoot_power = 255;
	}
	else{
		m_shoot_power = (255 * m_shoot_power / 100);
	}
#ifdef DEBUG
	std::cout << "shoot_regularization(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "shoot byte(hex): " << (m_shoot_power) << std::endl;
	std::cout << std::endl;
#endif
}

void BaseController::mDriverSetting()
{
// TODO
    m_en_stop = 0;
    if(mb_close){
        m_motorCurrPWM.w1 = 0;
        m_motorCurrPWM.w2 = 0;
        m_motorCurrPWM.w3 = 0;
    }else{
        m_en_stop += (fabs(m_motorCurrPWM.w1) >= MIN_PWM)?  0x80 : 0;
        m_en_stop += (fabs(m_motorCurrPWM.w2) >= MIN_PWM)?  0x40 : 0;
        m_en_stop += (fabs(m_motorCurrPWM.w3) >= MIN_PWM)?  0x20 : 0;
        m_en_stop += (fabs(m_motorCurrPWM.w1) == MIN_PWM)?  0x10 : 0;
        m_en_stop += (fabs(m_motorCurrPWM.w2) == MIN_PWM)?  0x08 : 0;
        m_en_stop += (fabs(m_motorCurrPWM.w3) == MIN_PWM)?  0x04 : 0;

    }
}

void BaseController::mBaseControl()
{
// TODO
    m_Motor1.SetSpeed(m_motorCommandRPM.w1, m_motorCurrRPM.w1);
    m_Motor2.SetSpeed(m_motorCommandRPM.w2, m_motorCurrRPM.w2);
    m_Motor3.SetSpeed(m_motorCommandRPM.w3, m_motorCurrRPM.w3);

    m_motorTarRPM.w1 = m_Motor1.MotorControl();
    m_motorTarRPM.w2 = m_Motor2.MotorControl();
    m_motorTarRPM.w3 = m_Motor3.MotorControl();

    m_motorCurrPWM.w1 = mPWMRegularization(m_Motor1.GetCurrPWM());
    m_motorCurrPWM.w2 = mPWMRegularization(m_Motor2.GetCurrPWM());
    m_motorCurrPWM.w3 = mPWMRegularization(m_Motor3.GetCurrPWM());

//    m_motorTarPWM.w1 =  RPM2PWM(m_motorTarRPM.w1);
//    m_motorTarPWM.w2 =  RPM2PWM(m_motorTarRPM.w2);
//    m_motorTarPWM.w3 =  RPM2PWM(m_motorTarRPM.w3);
}

void BaseController::mInverseKinematics()
{
    double cmd1, cmd2, cmd3;
	cmd1 = m_baseCommand.x*(-1)+m_baseCommand.y*(-0.5)+m_baseCommand.yaw*(-1);
	cmd2 = m_baseCommand.x*(1)+m_baseCommand.y*(-0.5)+m_baseCommand.yaw*(-1);
	cmd3 = m_baseCommand.x*(0)+m_baseCommand.y*(1)+m_baseCommand.yaw*(-1);
    m_motorCommandRPM.w1 = cmd1 / 100 * MAX_MOTOR_RPM;
    m_motorCommandRPM.w2 = cmd2 / 100 * MAX_MOTOR_RPM;
    m_motorCommandRPM.w3 = cmd3 / 100 * MAX_MOTOR_RPM;
#ifdef DEBUG
#endif
}

void BaseController::mForwardKinematics()
{
	double x,y;
	double yaw=0;
	int round=0;
	m_baseOdometry.x = ( m_baseRX.w1 * (-0.5774) + m_baseRX.w2 * (0.5774) + m_baseRX.w3 * (0)) * 2 * M_PI * wheel_radius / 26 / 2000;
	m_baseOdometry.y = ( m_baseRX.w1 * (-0.3333) + m_baseRX.w2 * (-0.3333) + m_baseRX.w3 * (0.6667)) * 2 * M_PI * wheel_radius / 26 / 2000;
	m_baseOdometry.yaw = (-1) * (m_baseRX.w1 * (1.6667) + m_baseRX.w2 * (1.6667) + m_baseRX.w3 * (1.6667))  * 2 *M_PI* wheel_radius / 2000 / 26;
}

void BaseController::mTrajectory()
{
// TODO 
//    odo.traj.x += odo.vel.x*cos(odo.vel.yaw)-odo.vel.y*sin(odo.vel.yaw);
//    odo.traj.y += odo.vel.x*sin(odo.vel.yaw)+odo.vel.y*cos(odo.vel.yaw);
//    odo.traj.yaw += odo.vel.yaw;
}

bool BaseController::GetBaseFlag()
{
    if(mb_base){
        mb_base = false;
        return true;

    }else
        return false;
}

bool BaseController::GetErrFlag()
{
// TODO
    return true;
}

uint8_t* BaseController::GetPacket()
{
    return serial_data;
}

MotorSpeed BaseController::GetCurrRPM()
{
    return m_motorCurrRPM;
}

MotorSpeed BaseController::GetCurrPWM()
{
    return m_motorCurrPWM;
}

MotorSpeed BaseController::GetTarRPM()
{
    return m_motorTarRPM;
}

MotorSpeed BaseController::GetTarPWM()
{
    return m_motorTarPWM;
}

void BaseController::Send(const RobotCommand &CMD)
{
#ifdef DEBUG
#endif
    m_baseCommand = CMD;
    mCommandRegularization();
    mInverseKinematics();
}

void BaseController::SetSingle(int number, int16_t rpm)
{
// TODO
    switch(number){
        case 1:
            m_motorCommandRPM.w1 = rpm;
            m_motorCommandRPM.w2 = 0;
            m_motorCommandRPM.w3 = 0;
            break;
        case 2:
            m_motorCommandRPM.w2 = rpm;
            m_motorCommandRPM.w1 = 0;
            m_motorCommandRPM.w3 = 0;
            break;
        case 3:
            m_motorCommandRPM.w3 = rpm;
            m_motorCommandRPM.w1 = 0;
            m_motorCommandRPM.w2 = 0;
            break;
        default:
            m_motorCommandRPM.w1 = 0;
            m_motorCommandRPM.w2 = 0;
            m_motorCommandRPM.w3 = 0;
            break;
    }
}

void BaseController::SetTriple(int16_t rpm1, int16_t rpm2, int16_t rpm3)
{
// TODO
    m_motorCommandRPM.w1 = rpm1;
    m_motorCommandRPM.w2 = rpm2;
    m_motorCommandRPM.w3 = rpm3;
}

void BaseController::SetEnable()
{
    mb_enable = true;
}

void BaseController::SetStop()
{
    mb_stop = true;
}

void BaseController::Close()
{
    mb_close = true;
    mDriverSetting();
    mCsslSend2FPGA();
}

//RobotCommand BaseController::GetOdoRobot()
//{
//// TODO
//	return RobotCommand();
//}
//
//RobotCommand BaseController::GetTraj()
//{
//// TODO
//	return RobotCommand();
//}

void BaseController::ShowCsslSend()
{
    printf("**************************\n");
    printf("* ****** mcssl_send ******\n");
    printf("**************************\n");
    printf("head1: %x\n", (m_baseTX.head1));
    printf("head2: %x\n", (m_baseTX.head2));
    printf("w1_h: %x\n", (m_baseTX.w1_h));
    printf("w1_l: %x\n", (m_baseTX.w1_l));
    printf("w2_h: %x\n", (m_baseTX.w2_h));
    printf("w2_l: %x\n", (m_baseTX.w2_l));
    printf("w3_h: %x\n", (m_baseTX.w3_h));
    printf("w3_l: %x\n", (m_baseTX.w3_l));
    printf("enable_stop: %x\n", (m_baseTX.enable_stop));
    printf("shoot: %x\n", (m_baseTX.shoot));
    printf("crc16-1: %x\n", (m_baseTX.crc_16_1));
    printf("crc16-2: %x\n", (m_baseTX.crc_16_2));
    printf("crc16: %x\n", (m_baseTX.crc_16_1 << 8) + (m_baseTX.crc_16_2));
    printf("w1: %x\n", ((m_baseTX.w1_h) << 8) + (m_baseTX.w1_l));
    printf("w2: %x\n", ((m_baseTX.w2_h) << 8) + (m_baseTX.w2_l));
    printf("w3: %x\n", ((m_baseTX.w3_h) << 8) + (m_baseTX.w3_l));
}

void BaseController::ShowCsslCallback()
{
    printf("Motor states: duration %d\n", (int)m_baseRX.duration);
    printf("packet size: %d\n", m_baseRX.size);
    printf("motor1 speed: %d\n", m_baseRX.w1);
    printf("motor2 speed: %d\n", m_baseRX.w2);
    printf("motor3 speed: %d\n", m_baseRX.w3);
    std::cout << "***eror message: " << m_baseRX.error << std::endl;
    std::cout << "***eror times: " << m_baseRX.error_times;
    printf("\n\n");
}

void BaseController::ShowCommand()
{
	std::cout << "Send\n";
    printf("x command: %f", m_baseCommand.x);
    printf("y command: %f", m_baseCommand.y);
    printf("yaw command: %f", m_baseCommand.yaw);
    printf("shoot power: %d\n", m_baseCommand.shoot_power);
    printf("hold ball: %d\n", m_baseCommand.hold_ball);
    printf("remote: %d\n\n", m_baseCommand.remote);
}

void BaseController::ShowSerialPacket()
{
    printf("*****Buffer data*****\n");
    
    printf("serialData.head: %d, serialData.rear: %d, duration: %d\n", ms_serialData.head, ms_serialData.rear, (int)m_baseRX.duration);
    for(int i = 0; i < BUFFER_SIZE; i++){
        if(i%10==0)printf("\n");
        printf("%x ", ms_serialData.data[i]);
    }
    printf("\n");
    printf("*****Serial data: length: %d\n", ms_length);
    for(int i = 0; i < RX_PACKAGE_SIZE; i++){
        printf("%x ", serial_data[i]);
    }
    printf("\n\n");


}
