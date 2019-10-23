#include <cstdio>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <highgui.h>
#include <vector>
#include <deque>
#include <math.h>
#include <signal.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <fstream>
#include "vision/center.h"
#include "vision/black.h"
#include "vision/bin.h"
#define PI 3.14159265
#define FRAME_COLS 659 //width  x659
#define FRAME_ROWS 493 //height y493
#define VISION_TOPIC "camera/image_raw"
#define YAML_PATH ros::package::getPath("vision") + "/config/FIRA.yaml"
#define IMAGE "/src/vision/1.bmp"

typedef unsigned char BYTE;
using namespace cv;
using namespace std;

struct DetectedObject
{
    DetectedObject();
    void Reset();
    int dis_max;     //pix
    int dis_min;     //pix
    int ang_max;     //pix
    int ang_min;     //pix
    int x;           //pix
    int y;           //pix
    int angle;       //pix
    double distance; //pix
    int size;
};

class NodeHandle
{
  protected:
	void Readyaml();
	NodeHandle();
	void Parameter_getting();
	//========================================
	void AngleLUT();
	vector<double> Angle_sin;
	vector<double> Angle_cos;
	int Frame_Area(int coordinate, int range);
	int Angle_Adjustment(int angle);
    int Angle_Interval(int radius);
	//================center==================
	int CenterXMsg;
	int CenterYMsg;
	int InnerMsg;
	int OuterMsg;
	int FrontMsg;
	double Camera_HighMsg;
    //================scan====================
    int Angle_Near_GapMsg;
    int Magn_Near_GapMsg;
    int Magn_Near_StartMsg;
    int Magn_Middle_StartMsg;
    int Magn_Far_StartMsg;
    int Magn_Far_EndMsg;
    int Dont_Search_Angle_1Msg;
    int Dont_Search_Angle_2Msg;
    int Dont_Search_Angle_3Msg;
    int Angle_range_1Msg;
    int Angle_range_2_3Msg;
    int Unscaned_Angle[8];
    void Set_Unscaned_Angle();
	//================black===================
	int BlackGrayMsg;
	int BlackAngleMsg;
	std_msgs::Int32MultiArray blackdis;
	//==============distance==================
	double camera_f(double Omni_pixel);
	double Omni_distance(double pixel_dis);
    void pub_obstacle(vector<DetectedObject> obstacle);
    void pub_pixel_obstacle(vector<DetectedObject> obstacle);
    void Pub_obstacleframe(Mat frame);

  private:
	ros::NodeHandle nh;
	ros::Subscriber save_sub;
	void SaveButton_setting(const vision::bin msg);
	int SaveButton;
	//==============black====================
	void blackcall(const vision::black msg);
    ros::Publisher obstacle_pub;
    ros::Publisher pixel_obstacle_pub;
    ros::Publisher obstacleframe_pub;
    int Strategy_Angle(int angle);
};
