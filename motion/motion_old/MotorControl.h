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
#include "../cssl/cssl.h"
#include "../cssl/port.h"
#include "../cssl/cssl.c"
//#include "cssl/info.h"
#define VERSION "by Lain-Jinn Hwang\n"
#include "math.h"
//using std::string;
//====motor define=====
#define ROBOT_RADIUS 0.15
#define WheelRadius  0.05
#define RPM_Max      6940
#define Gear         26
#define PWM_Range    127.0
#define PWM_Limit_Percent_Min   0.1
#define PWM_Limit_Percent_Max   0.9
#define RPM_Offset 0
#define PWM_MIN 20.0
#define PWM_MAX 103
#define x_inv
#define y_inv
#define yaw_inv 2.3251


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
geometry_msgs::Twist locationfeedback;
unsigned char dir_1,dir_2,dir_3;

void localization(geometry_msgs::Twist motorFB)
{
    static double tmp_x=0,tmp_y=0;
    static double FB_x=0,FB_y=0,FB_yaw=0;
    double motorFB_x = motorFB.linear.x;
    double motorFB_y = motorFB.linear.y;
    double motorFB_yaw = motorFB.angular.z;

    FB_x += (motorFB_x-tmp_x)*cos(motorFB_yaw)-(motorFB_y-tmp_y)*sin(motorFB_yaw);
    FB_y += (motorFB_y-tmp_y)*cos(motorFB_yaw)+(motorFB_x-tmp_x)*sin(motorFB_yaw);
    FB_yaw = motorFB_yaw;
    tmp_x = motorFB_x;
    tmp_y = motorFB_y;
    locationfeedback.linear.x = FB_x;
    locationfeedback.linear.y = FB_y;
    locationfeedback.angular.z = FB_yaw;
}

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
    i=i+length;
    if((i>14))i=i%15;

//    printf("feedback:\n");
//    for(int oo=0;oo<15;oo++){
//        printf("%x ,",feedback[oo]);
//    }
//    printf("\n");

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
//    printf("findpacket %d\n",findpacket);
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
    odom_x =(pls1*(-0.3333)+pls2*(-0.3333)+pls3*(0.6667))*2*M_PI*WheelRadius/(26)/2000;
    odom_y =(pls1*(0.5774) + pls2*(-0.5774) + pls3*(0))*2*M_PI*WheelRadius/26/2000;
//    odom_yaw =(pls1*(0.21231) + pls2*(0.21231) + pls3*(0.21231))/2000/26;
    odom_yaw =(pls1*(yaw_inv) + pls2*(yaw_inv) + pls3*(yaw_inv))*2*M_PI*WheelRadius/2000/26;
    int round;
    round = odom_yaw/(2*M_PI);
    odom_yaw = (odom_yaw-round*2*M_PI);
//    printf("round = %d\n");
//    printf("length_x=%lf , length_y=%lf , length_r=%lf\npls_1=%d,pls_2=%d, pls_3=%d \n\n",odom_x,odom_y,odom_yaw,pls1,pls2,pls3);
//    printf("\n=====================\n");

    motionfeedback.linear.x = odom_x;
    motionfeedback.linear.y = odom_y;
    motionfeedback.angular.z = odom_yaw;
    localization(motionfeedback);
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
        serial=cssl_open(devs, mcssl_callback, 0, 115200, 8, 0, 1);
    }
    if (!serial){
        devs="/dev/ttyUSB1";
        serial=cssl_open(devs, mcssl_callback, 0, 115200, 8, 0, 1);
    }
    if (!serial){
        devs="/dev/ttyUSB2";
        serial=cssl_open(devs, mcssl_callback, 0, 115200, 8, 0, 1);
    }
    if (!serial){
        devs="/dev/ttyUSB3";
        serial=cssl_open(devs, mcssl_callback, 0, 115200, 8, 0, 1);
    }
    if (!serial){
        devs="/dev/ttyUSB4";
        serial=cssl_open(devs, mcssl_callback, 0, 115200, 8, 0, 1);
    }
    if (!serial){
        devs="/dev/ttyUSB5";
        serial=cssl_open(devs, mcssl_callback, 0, 115200, 8, 0, 1);
    }
    if (!serial){
        devs="/dev/ttyUSB0";
        serial=cssl_open(devs, mcssl_callback, 0, 115200, 8, 0, 1);
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

void mcssl_send2motor(double w1, double w2,double w3,char en1,char en2,char en3,char shoot){
    double stop1,stop2,stop3;
    char w1_dir = (w1 > 0)  ? 0x80 : 0;
    char w2_dir = (w2 > 0)  ? 0x80 : 0;
    char w3_dir = (w3 > 0)  ? 0x80 : 0;
    char i;

    for(i=0;i<4;i++){
//        printf("w1 = %f,w2 = %f,w3 = %f\n",w1,w2,w3);
//        printf("abs(w1) = %d,abs(w2) = %d,abs(w3) = %d\n\n",abs(w1),abs(w2),abs(w3));
        if((abs(w1)>100)||(abs(w2)>100)||(abs(w3)>100)){
            w1 = w1*0.9;
            w2 = w2*0.9;
            w3 = w3*0.9;
        }else{
            break;
        }
    }

    if(w1!=0 ) {w1 = (w1>0)? (w1+12.7):(w1-12.7);}
    if(w2!=0 ) {w2 = (w2>0)? (w2+12.7):(w2-12.7);}
    if(w3!=0 ) {w3 = (w3>0)? (w3+12.7):(w3-12.7);}

    if(abs(w1)<13 ) w1 = 0;
    if(abs(w2)<13 ) w2 = 0;
    if(abs(w3)<13 ) w3 = 0;

//    printf("2.\tw1 = %f ; w2 = %f ; w3 = %f\n",w1,w2,w3);
    for(i=0;i<10;i++){
        if((abs(w1)>115)||(abs(w2)>115)||(abs(w3)>115)){
            w1 = w1*0.9;
            w2 = w2*0.9;
            w3 = w3*0.9;
        }else{
            break;
        }
    }

//    printf("3.\tw1 = %f ; w2 = %f ; w3 = %f\n",w1,w2,w3);
//    stop1 = (w1!=1)?0:1;
//    stop2 = (w2!=1)?0:1;
//    stop3 = (w3!=1)?0:1;
    stop1 = 0;
    stop2 = 0;
    stop3 = 0;

    en1 = (w1!=0)?1:0;
    en2 = (w2!=0)?1:0;
    en3 = (w3!=0)?1:0;

//    printf("w1 = %f\n",w1);
//    printf("w2 = %f\n",w2);
//    printf("w3 = %f\n",w3);


    char w1_byte = (char) abs(w1);
    char w2_byte = (char) abs(w2);
    char w3_byte = (char) abs(w3);

   char en_byte = en1*128+en2*64+en3*32+stop1*16+stop2*8+stop3*4;
//	char en_byte = en1*128+en2*64+en3*32;
    en_byte += shoot;
    if(abs(w1)>114.3)w1_byte=0;
    if(abs(w2)>114.3)w2_byte=0;
    if(abs(w3)>114.3)w3_byte=0;
//    printf("===>w1:%0.2f w2:%0.2f w3:%0.2f en1:%0.2f en2:%0.2f en3:%0.2f<===\n\n",w1,w2,w3,en1,en2,en3);
//    printf("=>w1_byte:%d w2_byte:%d w3_byte:%d en_byte:%x<= \n",w1_byte,w2_byte,w3_byte,en_byte);
//    printf("shoot = %d\n",shoot);
    unsigned char check;
    cssl_putchar(serial,0xff);
    cssl_putchar(serial,0xfa);

    cssl_putchar(serial,w1_byte + w1_dir);
    cssl_putchar(serial,w2_byte + w2_dir);
    cssl_putchar(serial,w3_byte + w3_dir);
    cssl_putchar(serial,en_byte);

}
geometry_msgs::Twist getmotorfeedback()
{
    return motionfeedback;
}
geometry_msgs::Twist getlocation()
{
    return locationfeedback;
}
