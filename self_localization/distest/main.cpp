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
#define N_PARTICLE 600

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

using namespace std;
using namespace cv;

int robot_angle = 0;
bool onclick = false;
Mat self_map(FIELD_HEIGHT+50*2, FIELD_WIDTH+50*2, CV_8UC3, Scalar(0,150,0));
Mat self_map_=self_map.clone();
double mouse_x=self_map.cols/2, mouse_y=self_map.rows/2;
int key = 0;
int ROBOT_SIZE = 20;
vector<int> point;
double robot_point[2] = {0,0};
int angle = 0;
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
void draw(){
	int img_x = 750;
	int img_y = 550;
	int angle_bet = 4;
	int self_robot_pos_x_img = mouse_x;
	int self_robot_pos_y_img = img_y - mouse_y;
	int scan_x;
	int scan_y;
	int operating_dis = 300;
    int robot_speed = 2;
    
	vector<int> white;

	Mat visual_map(550, 750, CV_8UC3, Scalar(0,0,0));
   
    self_map_=self_map.clone();
    int map_angle;
    if(key == 119){//w
        //mouse_y-=robot_speed;
        map_angle=robot_angle;
        if(map_angle>360)map_angle-=360;
        else if(map_angle<0)map_angle+=360;
        mouse_x+=robot_speed*cos(map_angle*DEG2RAD);
        mouse_y-=robot_speed*sin(map_angle*DEG2RAD);
        robot_point[1]+=robot_speed;
    }
    else if(key == 115){//s
        //mouse_y+=robot_speed;
        map_angle=robot_angle+180;
        if(map_angle>360)map_angle-=360;
        else if(map_angle<0)map_angle+=360;
        mouse_x+=robot_speed*cos(map_angle*DEG2RAD);
        mouse_y-=robot_speed*sin(map_angle*DEG2RAD);
        robot_point[1]-=robot_speed;
    }
    else if(key == 97){//a
        //mouse_x-=robot_speed;
        map_angle=robot_angle+90;
        if(map_angle>360)map_angle-=360;
        else if(map_angle<0)map_angle+=360;
        mouse_x+=robot_speed*cos(map_angle*DEG2RAD);
        mouse_y-=robot_speed*sin(map_angle*DEG2RAD);
        robot_point[0]-=robot_speed;
    }
    else if(key == 100){//d
        //mouse_x+=robot_speed;
        map_angle=robot_angle-90;
        if(map_angle>360)map_angle-=360;
        else if(map_angle<0)map_angle+=360;
        mouse_x+=robot_speed*cos(map_angle*DEG2RAD);
        mouse_y-=robot_speed*sin(map_angle*DEG2RAD);
        robot_point[0]+=robot_speed;
    }
    else if(key == 113){//q
        robot_angle+=2;
        if(robot_angle>=360)robot_angle-=360;
    }
    else if(key == 101){//e
        robot_angle-=2;
        if(robot_angle<0)robot_angle+=360;
    }

    point.clear();
	for(int angle = 0; angle < 360; angle+=angle_bet){
        auto count =0;
        int angle_=robot_angle+angle;
        if(angle_>=360)angle_-=360;
        if(angle_<0)angle_+=360;
		for(int dis = ROBOT_SIZE; dis < operating_dis+1; dis+=6){
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
				}else{
                    if(angle_==robot_angle){
                         circle(self_map_, Point(scan_x, scan_y), 0.5, Scalar(0, 0, 255), -1);
                    }
                    else
                        circle(self_map_, Point(scan_x, scan_y), 0.5, Scalar(100, 100, 100), -1);
				}
			}
		}
	}
    //resize(visual_map,visual_map,Size(visual_map.cols*0.8,visual_map.rows*0.8),0,0,INTER_LINEAR);
    circle(visual_map, Point(visual_map.cols/2, visual_map.rows/2), 5, Scalar(0, 0, 255), 1);
    line(visual_map, Point(visual_map.cols/2,visual_map.rows/2), Point(visual_map.cols/2+5,visual_map.rows/2), Scalar(0,0,255), 1);
	imshow("visual_map", visual_map);
	key = waitKey(10);
}



int main(int argc,char **argv){
	ros::init(argc, argv, "distest");
	ros::NodeHandle nh;

    ros::Publisher sensor_pub = nh.advertise<std_msgs::Int32MultiArray>("/vision/mcl/WhiteRealDis", 1);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/motion/motionFB", 1);
    ros::Publisher imu_pub = nh.advertise<imu_3d::inertia>("/imu_3d", 1);

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
    
    //imshow("self_map222",self_map);
    //waitKey(10);
    ros::Rate loop_rate(30); //Image transfer speed(hz)
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
		key = waitKey(100);
		if (onclick)
		{
			cout << "(" << mouse_x-self_map_.cols/2 << "," << mouse_y-self_map_.rows/2 << ")"<< endl;
			onclick = false;
		}
		draw();
        //==========
        //robot_point[0]=mouse_x-self_map.cols/2;
        //robot_point[1]=mouse_y-self_map.rows/2;
        angle = 360-robot_angle;
        if(angle>360){
            angle-=360;
        }
        else if(angle<0){
            angle+=360;
        }
         angle = angle+90;
        if(angle>360){
            angle-=360;
        }
        else if(angle<0){
            angle+=360;
        }
        point_msg.data = point;
        vel_msg.linear.x = robot_point[0]/100;
        vel_msg.linear.y = robot_point[1]/100;
        vel_msg.angular.z = angle;
        imu_msg.yaw=angle*DEG2RAD;
        //cout<<angle<<"   "<<angle*DEG2RAD<<endl;

        sensor_pub.publish(point_msg);
        vel_pub.publish(vel_msg);
        imu_pub.publish(imu_msg);

        ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


