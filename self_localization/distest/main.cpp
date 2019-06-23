#include <cstdio>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "imu_3d/inertia.h"

#define TEST "/home/kym/kym/src/vision/distest/test3.png"
#define DEG2RAD  M_PI/180
#define RAD2DEG 180/M_PI

#define FIELD_WIDTH 600
#define FIELD_HEIGHT 400
#define XLINE1 300
#define XLINE2 XLINE1-40
#define XLINE3 XLINE1-75
#define XLINE4 0
#define XLINE5 -(XLINE3)
#define XLINE6 -(XLINE2)
#define XLINE7 -(XLINE1)
#define XLINE8 75
#define XLINE9 -XLINE8
#define YLINE1 200
#define YLINE2 100
#define YLINE3 80
#define YLINE4 -(YLINE3)
#define YLINE5 -(YLINE2)
#define YLINE6 -(YLINE1)
#define CENTER_RADIUS 60
#define CENTER_RADIUS2 25

#define ROBOT_SIZE  20//(cm)
#define ROBOT_SPEED_MAX 1.52//(m/s)
#define ROBOT_ANGLE_SPEED_MAX 360//(degree/s)
using namespace std;
using namespace cv;

bool onclick = false;
Mat self_map(FIELD_HEIGHT+50*2, FIELD_WIDTH+50*2, CV_8UC3, Scalar(0,150,0));
Mat self_map_=self_map.clone();
double mouse_x=self_map.cols/2, mouse_y=self_map.rows/2;
int key = 0;

vector<int> point;
double robot_point[2] = {0,0};
double robot_angle = 0;
//double move[3] = {0,0,0};

//vector<vector<int> > robot_point;
void onMouse(int Event, int x, int y, int flags, void *param)
{
	if (Event == CV_EVENT_LBUTTONDOWN)
	{
		mouse_x = x;
		mouse_y = y;
		onclick = true;
	}
    
}
int color(Mat input, int y, int x, int i){
	int _color = input.data[(y * input.cols + x) * 3 + i];
	return _color;
}
void drawmap(Mat &input, int y, int x ,int cor){
    if(x<input.cols&&x>0&&y>0&&y<input.rows){
	    input.data[(y * input.cols + x) * 3 + 0] = cor;
	    input.data[(y * input.cols + x) * 3 + 1] = cor;
	    input.data[(y * input.cols + x) * 3 + 2] = cor;
    }
}
double camera_f(double Omni_pixel)
{
    double m = (Omni_pixel * 0.0099) / 60; // m = H1/H0 = D1/D0    D0 + D1 = 180
    double D0 = 180 / (1 + m);             // D1 = m   *D0
    double D1 = 180 / (1 + (1 / m));       // D0 = 1/m *D1
    double f = 1 / (1 / D0 + 1 / D1);
    //ROS_INFO("m = %f D0 = %f D1 = %f F = %f",m,D0,D1,f);
    return D1;
}
double Omni_distance(double pixel_dis)
{
    double Camera_HighMsg = 650;
    double OuterMsg = 228;
//=========================
    double Z = -1 * Camera_HighMsg; //Camera_HighMsg=650mm;
    //double c  =  D0/2;
    double c = 83.125;
    double b = c * 0.8722;
    double f = camera_f(OuterMsg * 2 * 0.9784);
    double r = atan2(f, pixel_dis * 0.0099);
    double dis = Z * (pow(b, 2) - pow(c, 2)) * cos(r) / ((pow(b, 2) + pow(c, 2)) * sin(r) - 2 * b * c) * 0.1;
    if (dis < 0 || dis > 999)
    {
        dis = 999;
    }
    //ROS_INFO("%f %f %f %f",Z,c,r,dis);
    return dis;
}
vector<int> to_omni(int X, int Y)
{
    double Camera_HighMsg = 650;
    double OuterMsg = 228;
//=========================
    vector<int> xy;
    double Z = -1 * Camera_HighMsg; //Camera_HighMsg=650mm;
    //double c  =  D0/2;
    double c = 83.125;
    double b = c * 0.8722;
    double f = camera_f(OuterMsg * 2 * 0.9784);
    //double r = atan2(f, pixel_dis * 0.0099);
    X=X*10;
    Y=Y*10;
    int x = 120*X*f*(pow(b, 2) - pow(c, 2))/((pow(b, 2) + pow(c, 2)) * Z - 2 * b * c*sqrt(pow(X, 2) + pow(Y, 2)+pow(Z, 2)));
    int y = 120*Y*f*(pow(b, 2) - pow(c, 2))/((pow(b, 2) + pow(c, 2)) * Z - 2 * b * c*sqrt(pow(X, 2) + pow(Y, 2)+pow(Z, 2)));
    //cout<<x<<endl;
    //cout<<y<<endl;
    xy.push_back(x);
    xy.push_back(y);
    //ROS_INFO("%f %f %f %f",Z,c,r,dis);
    return xy;
}
void draw(){
	int img_x = 750;
	int img_y = 550;
	int angle_bet = 4;
	int self_robot_pos_x_img = mouse_x;
	int self_robot_pos_y_img = img_y - mouse_y;
	int scan_x;
	int scan_y;
	int operating_dis = 300;
    
	vector<int> white;

	Mat visual_map(550, 750, CV_8UC3, Scalar(0,0,0));
    self_map_=self_map.clone();
//======================move the robot=======================
    int map_angle;
    int move = 2;//cm
    if(key == 119){//w
        //mouse_y-=move;
        map_angle=robot_angle;
        if(map_angle>360)map_angle-=360;
        else if(map_angle<0)map_angle+=360;
        mouse_x+=move*cos(map_angle*DEG2RAD);
        mouse_y-=move*sin(map_angle*DEG2RAD);
        robot_point[1]+=move;
    }
    else if(key == 115){//s
        //mouse_y+=move;
        map_angle=robot_angle+180;
        if(map_angle>360)map_angle-=360;
        else if(map_angle<0)map_angle+=360;
        mouse_x+=move*cos(map_angle*DEG2RAD);
        mouse_y-=move*sin(map_angle*DEG2RAD);
        robot_point[1]-=move;
    }
    else if(key == 97){//a
        //mouse_x-=move;
        map_angle=robot_angle+90;
        if(map_angle>360)map_angle-=360;
        else if(map_angle<0)map_angle+=360;
        mouse_x+=move*cos(map_angle*DEG2RAD);
        mouse_y-=move*sin(map_angle*DEG2RAD);
        robot_point[0]-=move;
    }
    else if(key == 100){//d
        //mouse_x+=move;
        map_angle=robot_angle-90;
        if(map_angle>360)map_angle-=360;
        else if(map_angle<0)map_angle+=360;
        mouse_x+=move*cos(map_angle*DEG2RAD);
        mouse_y-=move*sin(map_angle*DEG2RAD);
        robot_point[0]+=move;
    }
    else if(key == 113){//q
        robot_angle+=move;
        if(robot_angle>=360)robot_angle-=360;
    }
    else if(key == 101){//e
        robot_angle-=move;
        if(robot_angle<0)robot_angle+=360;
    }
//=========================================================
//===========================================================
/*
全向鏡模擬
    Mat test(493, 659, CV_8UC3, Scalar(0,0,0));
    circle(test, Point(test.cols/2,test.rows/2), 228, Scalar(255, 255, 255), -1);
    vector<int> xy;
    for(int i=0; i<self_map.rows; i++){
        for(int j=0; j<self_map.cols; j++){
            
            xy=to_omni(j-mouse_x,i-mouse_y);

            int dis=sqrt(pow(xy[0],2)+pow(xy[1],2));
            double angle = atan2(xy[1],xy[0]);
            angle = angle*RAD2DEG;
            double angle_=robot_angle+angle;
            int scan_x_ = (int)(dis*cos(angle_*DEG2RAD))+test.cols/2;
            int scan_y_ = (int)(dis*sin(angle_*DEG2RAD))+test.rows/2;

            xy[0]+=test.cols/2;
            xy[1]+=test.rows/2;

            if(xy[0]>test.rows||xy[0]<0)break;
            if(xy[1]>test.cols||xy[1]<0)break;
            int r,g,b;
            r=self_map.data[(i * self_map.cols + j) * 3 + 0];
	        g=self_map.data[(i * self_map.cols + j) * 3 + 1];
	        b=self_map.data[(i * self_map.cols + j) * 3 + 2];
            circle(test, Point(scan_x_, scan_y_), 1.2, Scalar(r, g, b), -1);
            //test.at<Vec3b>(scan_y_,scan_x_)= self_map.at<Vec3b>(i,j);
            
        }
    }

    imshow("test",test);
    //waitKey(10);
*/
//=========================================================
    point.clear();
    int line_count = 0;
	for(int angle = 0; angle < 360; angle+=angle_bet){
        auto count =0;
        int angle_=robot_angle+angle;
        if(angle_>=360)angle_-=360;
        if(angle_<0)angle_+=360;
		for(int dis = ROBOT_SIZE; dis < operating_dis+1; dis+=6){
            if(line_count%2==0&&dis<100)continue;
			scan_x = (int)(dis*cos(angle_*DEG2RAD))+self_robot_pos_x_img;
			scan_y = (int)(dis*sin(angle_*DEG2RAD))+self_robot_pos_y_img;
            int scan_x_ = (int)(dis*cos(angle*DEG2RAD))+visual_map.cols/2;
            int scan_y_ = (int)-(dis*sin(angle*DEG2RAD))+visual_map.rows/2;
			scan_y = 550 - scan_y;
			if(dis == operating_dis){
				//white.push_back(operating_dis);
				break;
			}else if(scan_x<0 || scan_x>=self_map.cols ||
				 scan_y<0 || scan_y>=self_map.rows){
				//white.push_back(operating_dis);		
				break;
			}
			else{
				if(color(self_map, scan_y, scan_x, 0) >= 250){
                    count++;
					white.push_back(dis);
                    circle(visual_map, Point(scan_x_, scan_y_), 2, Scalar(255, 255, 255), -1);
                    int x,y;
                    x=scan_x_-visual_map.cols/2;
                    y=scan_y_-visual_map.rows/2;
                    point.push_back(x);
                    point.push_back(y);
                    if(angle_==robot_angle){
                        circle(self_map_, Point(scan_x, scan_y), 3, Scalar(0, 0, 255), -1);
                    }else        
                        circle(self_map_, Point(scan_x, scan_y), 3, Scalar(0, 0, 0), -1);
                    break;
				}else{
                    if(angle_==robot_angle){
                         circle(self_map_, Point(scan_x, scan_y), 1, Scalar(0, 0, 255), -1);
                    }
                    else
                        circle(self_map_, Point(scan_x, scan_y), 1, Scalar(100, 100, 100), -1);
				}
			}
		}
        line_count++;
	}
    //resize(visual_map,visual_map,Size(visual_map.cols*0.8,visual_map.rows*0.8),0,0,INTER_LINEAR);
    circle(visual_map, Point(visual_map.cols/2, visual_map.rows/2), 5, Scalar(0, 0, 255), 1);
    line(visual_map, Point(visual_map.cols/2,visual_map.rows/2), Point(visual_map.cols/2+5,visual_map.rows/2), Scalar(0,0,255), 1);
	//imshow("visual_map", visual_map);
	//key = waitKey(10);
//=======================================================================================================
    
//=======================================================================================================
}
double cmd_x=0;
double cmd_y=0;
double cmd_w=0;
void velCallback(const geometry_msgs::Twist msg)
{
    //cout<<"msg.linear.x  "<<msg.linear.x<<endl;
    //cout<<"msg.linear.y  "<<msg.linear.y<<endl;
    //cout<<"msg.angular.z "<<msg.angular.z<<endl;
    
    cmd_x=msg.linear.x/100;
    cmd_y=msg.linear.y/100;
    cmd_w=msg.angular.z/100;
}
int to_omni_xy[500][700];
int main(int argc,char **argv){
	ros::init(argc, argv, "distest");
	ros::NodeHandle nh;

    ros::Publisher sensor_pub = nh.advertise<std_msgs::Int32MultiArray>("/vision/mcl/WhiteRealDis", 1);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/motion/motionFB", 1);
    ros::Publisher imu_pub = nh.advertise<imu_3d::inertia>("/imu_3d", 1);
    ros::Subscriber sub = nh.subscribe("/motion/cmd_vel", 1000, velCallback);

    line(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE1), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE1), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE6), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE6), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE1), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE6), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE1), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE6), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE4,self_map.rows/2+YLINE1), Point(self_map.cols/2+XLINE4,self_map.rows/2+YLINE6), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE2,self_map.rows/2+YLINE3), Point(self_map.cols/2+XLINE2,self_map.rows/2+YLINE4), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE6,self_map.rows/2+YLINE3), Point(self_map.cols/2+XLINE6,self_map.rows/2+YLINE4), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE3,self_map.rows/2+YLINE2), Point(self_map.cols/2+XLINE3,self_map.rows/2+YLINE5), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2+YLINE2), Point(self_map.cols/2+XLINE5,self_map.rows/2+YLINE5), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE2,self_map.rows/2+YLINE3), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE3), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE2,self_map.rows/2+YLINE4), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE4), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE3,self_map.rows/2+YLINE2), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE2), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE3,self_map.rows/2+YLINE5), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE5), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE6,self_map.rows/2+YLINE3), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE3), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE6,self_map.rows/2+YLINE4), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE4), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2+YLINE2), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE2), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2+YLINE5), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE5), Scalar(255,255,255), 3);
    circle(self_map, Point(self_map.cols/2,self_map.rows/2), CENTER_RADIUS, Scalar(255, 255, 255), 3);
    ellipse(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE1), Size(CENTER_RADIUS2,CENTER_RADIUS2), 0, 180, 270, Scalar(255,255,255), 3);
    ellipse(self_map, Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE1), Size(CENTER_RADIUS2,CENTER_RADIUS2), 0, 270, 360, Scalar(255,255,255), 3);
    ellipse(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE6), Size(CENTER_RADIUS2,CENTER_RADIUS2), 0, 90, 180, Scalar(255,255,255), 3);
    ellipse(self_map, Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE6), Size(CENTER_RADIUS2,CENTER_RADIUS2), 0, 0, 90, Scalar(255,255,255), 3);

    ellipse(self_map, Point(self_map.cols/2+XLINE3,self_map.rows/2), Size(CENTER_RADIUS,CENTER_RADIUS), 0, 90, 270, Scalar(255,255,255), 3);
    ellipse(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2), Size(CENTER_RADIUS,CENTER_RADIUS), 0, 0, 90, Scalar(255,255,255), 3);
    ellipse(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2), Size(CENTER_RADIUS,CENTER_RADIUS), 0, 270, 360, Scalar(255,255,255), 3);

    circle(self_map, Point(self_map.cols/2+XLINE8,self_map.rows/2+YLINE2), 3, Scalar(255, 255, 255), -1);
    circle(self_map, Point(self_map.cols/2+XLINE8,self_map.rows/2+YLINE5), 3, Scalar(255, 255, 255), -1);
    circle(self_map, Point(self_map.cols/2+XLINE9,self_map.rows/2+YLINE2), 3, Scalar(255, 255, 255), -1);
    circle(self_map, Point(self_map.cols/2+XLINE9,self_map.rows/2+YLINE5), 3, Scalar(255, 255, 255), -1);
    
    circle(self_map, Point(550,450), 3, Scalar(255, 255, 255), -1);

    rectangle(self_map, Point(self_map.cols/2+XLINE7-40,self_map.rows/2+YLINE4), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE3), Scalar(0,200,200), -1);
    rectangle(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE4), Point(self_map.cols/2+XLINE1+40,self_map.rows/2+YLINE3), Scalar(200,0,0), -1);
    
    //imshow("self_map",self_map);
    //waitKey(10);
    ros::Rate loop_rate(50); //Image transfer speed(hz)
	while(ros::ok()){
        std_msgs::Int32MultiArray point_msg;
        geometry_msgs::Twist vel_msg;
        imu_3d::inertia imu_msg;

		circle(self_map_, Point(mouse_x, mouse_y), ROBOT_SIZE, Scalar(255, 0, 0), 2);
        int front_lengh = 30;
        int x_ = mouse_x+front_lengh * cos(robot_angle * DEG2RAD);
        int y_ = mouse_y-front_lengh * sin(robot_angle * DEG2RAD);
        line(self_map_, Point(mouse_x,mouse_y), Point(x_,y_), Scalar(255,0,0), 2);
        //resize(self_map_,self_map_,Size(self_map_.cols*0.8,self_map_.rows*0.8),0,0,INTER_LINEAR);
		imshow("self_map",self_map_);
		setMouseCallback("self_map", onMouse, NULL);
		key = waitKey(10);
		if (onclick){
			cout << "(" << mouse_x-self_map_.cols/2 << "," << mouse_y-self_map_.rows/2 << ")"<< endl;
			onclick = false;
		}
//=====================simulation strategy velocity=========================
    
        static double start_time = ros::Time::now().toSec();
        double end_time = ros::Time::now().toSec();
        double dt = end_time-start_time;    
        
        static double robot_speed_x=0;
        static double robot_speed_y=0;
        static double robot_speed_w=0;

        double move_x=0;
        double move_y=0;
        double move_w=0;
        double speed_up=ROBOT_SPEED_MAX/0.5;
        double angle_speed_up=ROBOT_ANGLE_SPEED_MAX/1;
        double delay=1;

        if(dt<0.5&&dt>0){
            if(fabs(robot_speed_x)<fabs(ROBOT_SPEED_MAX*cmd_x)){
                if(ROBOT_SPEED_MAX*cmd_x>0){
                    if(robot_speed_x+speed_up*dt<=ROBOT_SPEED_MAX*cmd_x){
                        robot_speed_x+=speed_up*dt;
                    } else {
                        robot_speed_x=ROBOT_SPEED_MAX*cmd_x;
                    }
                }
                if(ROBOT_SPEED_MAX*cmd_x<0){
                     if(robot_speed_x-speed_up*dt>=ROBOT_SPEED_MAX*cmd_x){
                        robot_speed_x-=speed_up*dt;
                    } else {
                        robot_speed_x=ROBOT_SPEED_MAX*cmd_x;
                    }
                }
            }
            if(fabs(robot_speed_x)>fabs(ROBOT_SPEED_MAX*cmd_x)){
                if(ROBOT_SPEED_MAX*cmd_x>0){
                    if(robot_speed_x-speed_up*dt>=ROBOT_SPEED_MAX*cmd_x){
                        robot_speed_x-=speed_up*dt;
                    } else {
                        robot_speed_x=ROBOT_SPEED_MAX*cmd_x;
                    }
                }
                if(ROBOT_SPEED_MAX*cmd_x<0){
                     if(robot_speed_x+speed_up*dt<=ROBOT_SPEED_MAX*cmd_x){
                        robot_speed_x+=speed_up*dt;
                    } else {
                        robot_speed_x=ROBOT_SPEED_MAX*cmd_x;
                    }
                }
            }
            if(ROBOT_SPEED_MAX*cmd_x==0){
                if(robot_speed_x>0){
                    if(robot_speed_x-speed_up*dt>=ROBOT_SPEED_MAX*cmd_x){
                        robot_speed_x-=speed_up*dt;
                    }else{
                        robot_speed_x=ROBOT_SPEED_MAX*cmd_x;
                    }
                }else if(robot_speed_x<0){
                     if(robot_speed_x+speed_up*dt<=ROBOT_SPEED_MAX*cmd_x){
                        robot_speed_x+=speed_up*dt;
                     }else{
                        robot_speed_x=ROBOT_SPEED_MAX*cmd_x;
                     }
                }else{
                    robot_speed_x=0;
                }
            }
            if(fabs(robot_speed_y)<fabs(ROBOT_SPEED_MAX*cmd_y)){
                if(ROBOT_SPEED_MAX*cmd_y>0){
                    if(robot_speed_y+speed_up*dt<=ROBOT_SPEED_MAX*cmd_y){
                        robot_speed_y+=speed_up*dt;
                    }
                    else{
                        robot_speed_y=ROBOT_SPEED_MAX*cmd_y;
                    }
                }
                if(ROBOT_SPEED_MAX*cmd_y<0){
                    if(robot_speed_y-speed_up*dt>=ROBOT_SPEED_MAX*cmd_y){
                        robot_speed_y-=speed_up*dt;
                    } else {
                        robot_speed_y=ROBOT_SPEED_MAX*cmd_y;
                    }
                }
            }
            if(fabs(robot_speed_y)>fabs(ROBOT_SPEED_MAX*cmd_y)){
                if(ROBOT_SPEED_MAX*cmd_y>0){
                    if(robot_speed_y-speed_up*dt>=ROBOT_SPEED_MAX*cmd_y){
                        robot_speed_y-=speed_up*dt;
                    }
                    else{
                        robot_speed_y=ROBOT_SPEED_MAX*cmd_y;
                    }
                }
                if(ROBOT_SPEED_MAX*cmd_y<0){
                    if(robot_speed_y+speed_up*dt<=ROBOT_SPEED_MAX*cmd_y){
                        robot_speed_y+=speed_up*dt;
                    } else {
                        robot_speed_y=ROBOT_SPEED_MAX*cmd_y;
                    }
                }
            }
            if(ROBOT_SPEED_MAX*cmd_y==0){
                if(robot_speed_y>0){
                    if(robot_speed_y-speed_up*dt>=ROBOT_SPEED_MAX*cmd_y)
                        robot_speed_y-=speed_up*dt;
                    else
                        robot_speed_y=ROBOT_SPEED_MAX*cmd_y;
                }else if(robot_speed_y<0){
                     if(robot_speed_y+speed_up*dt<=ROBOT_SPEED_MAX*cmd_y)
                        robot_speed_y+=speed_up*dt;
                     else
                        robot_speed_y=ROBOT_SPEED_MAX*cmd_y;
                }else{  
                    robot_speed_y=0;
                }
            }
            robot_speed_w=ROBOT_ANGLE_SPEED_MAX*cmd_w;
            /*
            if(fabs(robot_speed_w)<fabs(ROBOT_ANGLE_SPEED_MAX*cmd_w)){
                if(ROBOT_ANGLE_SPEED_MAX*cmd_w>0){
                    if(robot_speed_w+angle_speed_up*dt<=ROBOT_ANGLE_SPEED_MAX*cmd_w)
                        robot_speed_w+=angle_speed_up*dt;
                    else
                        robot_speed_w=ROBOT_ANGLE_SPEED_MAX*cmd_w;
                }
                if(ROBOT_ANGLE_SPEED_MAX*cmd_w<0){
                     if(robot_speed_w-angle_speed_up*dt>=ROBOT_ANGLE_SPEED_MAX*cmd_w)
                        robot_speed_w-=angle_speed_up*dt;
                    else
                        robot_speed_w=ROBOT_ANGLE_SPEED_MAX*cmd_w;
                }
            }
            if(fabs(robot_speed_w)<fabs(ROBOT_ANGLE_SPEED_MAX*cmd_w)){
                if(ROBOT_ANGLE_SPEED_MAX*cmd_w>0){
                    if(robot_speed_w-angle_speed_up*dt>=ROBOT_ANGLE_SPEED_MAX*cmd_w)
                        robot_speed_w-=angle_speed_up*dt;
                    else
                        robot_speed_w=ROBOT_ANGLE_SPEED_MAX*cmd_w;
                }
                if(ROBOT_ANGLE_SPEED_MAX*cmd_w<0){
                     if(robot_speed_w+angle_speed_up*dt<=ROBOT_ANGLE_SPEED_MAX*cmd_w)
                        robot_speed_w+=angle_speed_up*dt;
                    else
                        robot_speed_w=ROBOT_ANGLE_SPEED_MAX*cmd_w;
                }
            }
            if(ROBOT_ANGLE_SPEED_MAX*cmd_w==0){
                if(robot_speed_w>0){
                    if(robot_speed_w-angle_speed_up*dt>=ROBOT_ANGLE_SPEED_MAX*cmd_w)
                        robot_speed_w-=angle_speed_up*dt;
                    else
                        robot_speed_w=ROBOT_ANGLE_SPEED_MAX*cmd_w;
                }else if(robot_speed_w<0){
                     if(robot_speed_w+angle_speed_up*dt<=ROBOT_ANGLE_SPEED_MAX*cmd_w)
                        robot_speed_w+=angle_speed_up*dt;
                     else
                        robot_speed_w=ROBOT_ANGLE_SPEED_MAX*cmd_w;
                }else{
                    robot_speed_w=0;
                }
             }
             */
        }else{
            robot_speed_x=0;
            robot_speed_y=0;
            robot_speed_w=0;
        }
        //cout<<"ROBOT_SPEED_MAX*cmd_x "<<ROBOT_SPEED_MAX*cmd_x<<endl;
        //cout<<"robot_speed_x "<<robot_speed_x<<endl;
        //cout<<"robot_speed_y "<<robot_speed_y<<endl;
        //cout<<"robot_speed_w "<<robot_speed_w<<endl;
        move_x=robot_speed_x*dt*100;
        move_y=robot_speed_y*dt*100;
        move_w=robot_speed_w*dt;

        //=====================

        mouse_x+=move_x*cos((robot_angle-90)*DEG2RAD);
        mouse_y-=move_x*sin((robot_angle-90)*DEG2RAD);
        robot_point[0]+=2*(move_x);
        mouse_x+=move_y*cos((robot_angle+0)*DEG2RAD);
        mouse_y-=move_y*sin((robot_angle+0)*DEG2RAD);
        robot_point[1]+=2*(move_y);
        //=====================
        robot_angle+=move_w;
        if(robot_angle>=360)robot_angle-=360;
        else if(robot_angle<0)robot_angle+=360;
        //=====================
        start_time=end_time;
//==============================================

		draw();
        //==========
        //robot_point[0]=mouse_x-self_map.cols/2;
        //robot_point[1]=mouse_y-self_map.rows/2;
        double output_angle = 360-robot_angle;
        if(output_angle>360){
            output_angle-=360;
        }
        else if(output_angle<0){
            output_angle+=360;
        }
        output_angle = output_angle+90;
        if(output_angle>360){
            output_angle-=360;
        }
        else if(output_angle<0){
            output_angle+=360;
        }
        point_msg.data = point;
        //cout<<"vel_msg.linear.x: "<<robot_point[0]/100<<endl;
        //cout<<"vel_msg.linear.y: "<<robot_point[1]/100<<endl;
        vel_msg.linear.x = robot_point[0]/100;
        vel_msg.linear.y = robot_point[1]/100;
        vel_msg.angular.z = 360-output_angle;
        imu_msg.yaw=output_angle*DEG2RAD;
        //cout<<imu_msg.yaw<<endl;
        //cout<<robot_speed_w*DEG2RAD<<endl;
        sensor_pub.publish(point_msg);
        vel_pub.publish(vel_msg);
        imu_pub.publish(imu_msg);

        ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


