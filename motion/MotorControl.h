//-------cssl include-------//
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "geometry_msgs/Twist.h"
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdio.h>
#include "cssl/cssl.h"
#include "cssl/port.h"
#include "cssl/cssl.c"
//#include "cssl/info.h"
#define VERSION "by Lain-Jinn Hwang\n"
#include "math.h"
//using std::string;
//====motor define=====
//#define ROBOT_RADIUS 0.15
#define ROBOT_RADIUS 1
#define WheelRadius  0.0508
#define RPM_Max      6940
#define Gear         26
#define PWM_Range    127.0
#define PWM_Limit_Percent_Min   0.1
#define PWM_Limit_Percent_Max   0.9
#define RPM_Offset 0
#define PWM_MIN 20.0
#define PWM_MAX 103
//#define Debug
//double pre_v_l=0;
//double pre_v_r=0;
//int feedback[8];
//int pls1,pls2,pls3;
//-------cssl variable-------//
char *echo ="\r";
cssl_t *serial;
//====================//
//   cssl callback    //
//====================//

unsigned char feedback[15];
double odom_x=0,odom_y=0,odom_yaw=0;
int pls1=0,pls2=0,pls3=0;

geometry_msgs::Twist motionfeedback;
unsigned char dir_1,dir_2,dir_3;
static void mcssl_callback(int id, uint8_t *buf, int length)
{
//    printf("test");
    static int i=0;
    int checknum;
    unsigned char pls1_1,pls1_2,pls1_3,pls1_4;
    unsigned char pls2_1,pls2_2,pls2_3,pls2_4;
    unsigned char pls3_1,pls3_2,pls3_3,pls3_4;
    bool findpacket=0;
//        printf("*buf=%x\n",*buf);
    //-----store packet
    feedback[i]=*buf;

//    printf("feedback[%d]=%d\n",i,feedback[i]);
//    printf("feedback[10]=%d,feedback[11]=%d,feedback[12]=%d,feedback[13]=%d\n",feedback[2],feedback[3],feedback[4],feedback[5]);
    i=i+length;
//    printf("length=%x\n",length);
//    printf("i=%x\n",i);
    if((i>14))i=i%15;
//    printf("feedback:\n");
//    for(int oo=0;oo<15;oo++){
//        printf("%x ,",feedback[oo]);
//    }
//    printf("\n");
    if(i == 14){
        for(int header=0;header<15;header++){
//                printf("feedback:\n");
//                for(int oo=0;oo<15;oo++){
//                    printf("%x ,",feedback[(header+oo)%15]);
//                }
//                printf("\n");
            if(feedback[header]!=0xff){continue;}
            else if(feedback[(header+1)%15]!=0xfa){continue;}
            else{
                if(header == 0){checknum=15;}
                else{checknum=header;}
                unsigned char checkcode = feedback[checknum-1];
                unsigned char checksum = 0;
                for(int check=2;check<14;check++){
                    checksum += feedback[(header+check)%15];
                }
//                printf("checksum = %x checknum = %x\n",checksum,checkcode);
                if(checksum != checkcode){continue;}
                else{
                    findpacket=1;
//                    printf("checksum = %x checknum = %x\n",checksum,checkcode);
//                    printf("feedback[14]=%x ,checkcode=%x\n",feedback[14],checkcode);
                    if(feedback[14]!=checkcode){
//                    printf("checksum = %x checknum = %x\n",checksum,checkcode);
                    for(int permutation=15;permutation<15-header;permutation--){
                        unsigned char tmp[2];
//                        printf("move:\n");
                        for(int move=0;move<15;move++){
                            tmp[1] = feedback[(header+move+1)%15];
                            tmp[0] = tmp[1];
                            feedback[(header+move+1)%15] = tmp[0];
//                            printf("%x ,",feedback[move]);
                        }
//                        printf("\n");
                    }
                    i = 0;
                    }else{break;}
                }
            }
        }
    }
//    static int aa=0;
//    static bool bb = 0;
//    static double  start;
//    if(bb == 0){
//        start = ros::Time::now().toSec();
//        bb = 1;
//    }
//    double now = ros::Time::now().toSec();
//    printf("[Feedback start time:%f]\n",now-start);
//    printf("%d ",aa++);
//    printf("feedback:\n");
//    for(int oo=0;oo<15;oo++){
//        printf("%x ,",feedback[oo]);
//    }
//    printf("\n");

    if((findpacket==1)&&(feedback[0]==0xff)&&(feedback[1]==0xfa)){
//            printf("feedback:\n");
//            for(int oo=0;oo<15;oo++){
//                printf("%x ,",feedback[oo]);
//            }
//            printf("\n");


        pls1_4 = feedback[2];   //motor1
        pls1_3 = feedback[3];
        pls1_2 = feedback[4];
        pls1_1 = feedback[5];

        pls2_4 = feedback[6];   //motor2
        pls2_3 = feedback[7];
        pls2_2 = feedback[8];
        pls2_1 = feedback[9];

        pls3_4 = feedback[10];  //motor3
        pls3_3 = feedback[11];
        pls3_2 = feedback[12];
        pls3_1 = feedback[13];
        pls1 = (pls1_4 << 24)+(pls1_3 << 16)+(pls1_2 << 8)+pls1_1;
        pls2 = (pls2_4 << 24)+(pls2_3 << 16)+(pls2_2 << 8)+pls2_1;
        pls3 = (pls3_4 << 24)+(pls3_3 << 16)+(pls3_2 << 8)+pls3_1;
    }else{
        pls1=pls1;
        pls2=pls2;
        pls3=pls3;
    }



//    printf("pls_1=%d \n",pls3);
    odom_x =(pls1*(-0.3333)+pls2*(-0.3333)+pls3*(0.6667))*2*M_PI*0.0508/(26)/2000;
    odom_y =(pls1*(0.5774) + pls2*(-0.5774) + pls3*(0))*2*M_PI*0.0508/26/2000;
    odom_yaw =(pls1*(0.21231) + pls2*(0.21231) + pls3*(0.21231))/2000/26;
//    printf("length_x=%f , length_y=%f , length_r=%f\npls_1=%d,pls_2=%d, pls_3=%d \n\n",odom_x,odom_y,odom_yaw,pls1,pls2,pls3);
    
    motionfeedback.linear.x = odom_x;
    motionfeedback.linear.y = odom_y;
    motionfeedback.angular.z = odom_yaw;
//    printf("mcssl_callback\n");
//    printf("id=%d ,buf=%x ,length=%d\n",id,*buf,length);
}


//====================//
//   cssl init        //
//====================//
int mcssl_init(/*const char *devs*/)
{
    char *devs;
    std::string port_name;

     cssl_start();

    // modify 19200 to desire value
    //serial=cssl_open(devs, mcssl_callback, 0, 115200, 8, 0, 1);
    if (!serial){
        devs="/dev/ttyUSB0";
        serial=cssl_open(devs, NULL/*mcssl_callback*/, 0, 115200, 8, 0, 1);
    }
    if (!serial){
        devs="/dev/ttyUSB1";
        serial=cssl_open(devs, NULL/*mcssl_callback*/, 0, 115200, 8, 0, 1);
    }
    if (!serial){
        devs="/dev/ttyUSB2";
        serial=cssl_open(devs, NULL/*mcssl_callback*/, 0, 115200, 8, 0, 1);
    }
    if (!serial){
        devs="/dev/ttyUSB3";
        serial=cssl_open(devs, NULL/*mcssl_callback*/, 0, 115200, 8, 0, 1);
    }
    if (!serial){
        devs="/dev/ttyUSB4";
        serial=cssl_open(devs, NULL/*mcssl_callback*/, 0, 115200, 8, 0, 1);
    }
    if (!serial){
        devs="/dev/ttyUSB5";
        serial=cssl_open(devs, NULL/*mcssl_callback*/, 0, 115200, 8, 0, 1);
    }
    if (!serial){
        devs="/dev/ttyUSB0";
        serial=cssl_open(devs, NULL/*mcssl_callback*/, 0, 115200, 8, 0, 1);
    }

    ROS_INFO("Initialize Motion with port=%s...",devs);
    if (!serial){
        printf("%s\n",cssl_geterrormsg());
        printf("---> Motion RS232 OPEN FAIL <---\n");
        fflush(stdout);
        return -1;
    }
    cssl_setflowcontrol(serial, 0, 0);


    return 1;
}

//====================//
//   cssl finish      //
//====================//
void mcssl_finish(){

    cssl_close(serial);
    cssl_stop();

}

void mcssl_send2motor(double w1, double w2,double w3,double en1,double en2,double en3,char shoot)
{

    unsigned char w1_dir = (w1 > 0)  ? 0x80 : 0;
    unsigned char w2_dir = (w2 > 0)  ? 0x80 : 0;
    unsigned char w3_dir = (w3 > 0)  ? 0x80 : 0;
    unsigned char checksum;
    char i;
    for(i=0;i<4;i++){
        if((fabs(w1)>100)||(fabs(w2)>100)||(fabs(w3)>100)){
            w1 = w1*0.9;
            w2 = w2*0.9;
            w3 = w3*0.9;
        }else{
            break;
        }
    }
    //160924, 12.7 to 13
    if(w1!=0 ) {w1 = (w1>0)? (w1+12.7):(w1-12.7);}
    if(w2!=0 ) {w2 = (w2>0)? (w2+12.7):(w2-12.7);}
    if(w3!=0 ) {w3 = (w3>0)? (w3+12.7):(w3-12.7);}

    if(fabs(w1)<13 ) w1 = 0;
    if(fabs(w2)<13 ) w2 = 0;
    if(fabs(w3)<13 ) w3 = 0;
//    printf("2.\tw1 = %f ; w2 = %f ; w3 = %f\n",w1,w2,w3);
    for(i=0;i<10;i++){
        if((fabs(w1)>115)||(fabs(w2)>115)||(fabs(w3)>115)){
            w1 = w1*0.9;
            w2 = w2*0.9;
            w3 = w3*0.9;
        }else{
            break;
        }
    }

//    printf("3.\tw1 = %f ; w2 = %f ; w3 = %f\n",w1,w2,w3);

    en1 = (w1!=0)?1:0;
    en2 = (w2!=0)?1:0;
    en3 = (w3!=0)?1:0;


//    printf("w1 = %f\n",w1);
//    printf("w2 = %f\n",w2);
//    printf("w3 = %f\n",w3);


    unsigned char w1_byte = (unsigned char) fabs(w1);
    unsigned char w2_byte = (unsigned char) fabs(w2);
    unsigned char w3_byte = (unsigned char) fabs(w3);

    unsigned char en_byte = (unsigned char) abs(en1*128+en2*64+en3*32);

    checksum =  w1_byte+w1_dir+
                w2_byte+w2_dir+
                w3_byte+w3_dir+
                en_byte+shoot;
#ifdef Debug
    ROS_INFO("Debug mode (MotorControl)\n");
    ROS_INFO("%d %d %d %d %d %d\n", w1_byte+w1_dir, w2_byte+w2_dir, w3_byte+w3_dir, en_byte, shoot,checksum);
#else
    cssl_putchar(serial,0xff);
    cssl_putchar(serial,0xfa);
    cssl_putchar(serial,w1_byte + w1_dir);
    cssl_putchar(serial,w2_byte + w2_dir);
    cssl_putchar(serial,w3_byte + w3_dir);
    cssl_putchar(serial,en_byte);
    cssl_putchar(serial,shoot);
    cssl_putchar(serial,checksum);
#endif
}
geometry_msgs::Twist getmotionfeedback()
{
    return motionfeedback;
}
