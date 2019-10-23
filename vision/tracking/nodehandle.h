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
#include <math.h>
#include <signal.h>
#include <fstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include "vision/center.h"
#include "vision/bin.h"
#define PI 3.14159265
#define FRAME_COLS 659 //width  x695
#define FRAME_ROWS 493 //height y493

#define YAML_PATH ros::package::getPath("vision")+"/config/FIRA.yaml"
#define BIN_PATH ros::package::getPath("vision") + "/config/HSVcolormap.bin"

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
    string LR;
};
class NodeHandle
{
public:
	NodeHandle();
	void Readyaml();
	void get_param();
//==========================================
	void AngleLUT();
	vector<double> Angle_sin;
	vector<double> Angle_cos;
//================center====================
	int CenterXMsg;
	int CenterYMsg;
	int InnerMsg;
	int OuterMsg;
	int FrontMsg;
    int FieldMsg;
	double Camera_HighMsg;
//================scan=====================
    int Strategy_Angle(int angle);
	int Frame_Area(int coordinate, int range);
	int Angle_Adjustment(int angle);
    int Angle_Interval(int radius);
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
//================color===================
    vector<BYTE> ColorFile();
    vector<BYTE> color_map;
	vector<int> HSV_red;
	vector<int> HSV_green;
//==============distance==================
	double camera_f(double Omni_pixel);
	double Omni_distance(double pixel_dis);
private:
	ros::NodeHandle nh;
	ros::Subscriber save_sub;
	void SaveButton_setting(const vision::bin msg);
    void Set_Unscaned_Angle();
	int SaveButton;
};

