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
#include <std_srvs/Empty.h>
#include <fstream>
#include "vision/parameterbutton.h"
#include "vision/camera.h"
#include "vision/center.h"
#include "vision/position.h"
#include "vision/dis.h"
#include "vision/scan.h"
#include "vision/color.h"
#include "vision/white.h"
#include "vision/black.h"
#include "vision/bin.h"
#define PI 3.14159265
#define FRAME_COLS 659 //width  x659
#define FRAME_ROWS 493 //height y493
#define REDITEM 0x01
#define GREENITEM 0x02
#define BLUEITEM 0x04
#define YELLOWITEM 0x08
#define WHITEITEM 0x10//WHITEITEM=robot
#define VISION_TOPIC "/camera/image_raw"
#define YAML_PATH ros::package::getPath("vision")+"/config/FIRA.yaml"
#define BIN_PATH ros::package::getPath("vision")+"/config/HSVcolormap.bin"
#define test "src/vision/1.bmp"

typedef unsigned char BYTE;
using namespace cv;
using namespace std;


class NodeHandle
{
protected:
	NodeHandle();
	void Readyaml();
	void Saveyaml();
	void Parameter_getting();
//==========================================
	int buttonMsg;
	void AngleLUT();
	vector<double> Angle_sin;
	vector<double> Angle_cos;
	void Pub_interface(Mat frame);
//================camera====================
	int fpsMsg;
	void get_campara();
	void set_campara(int value_ex);
//================center====================
	int CenterXMsg;
	int CenterYMsg;
	int InnerMsg;
	int OuterMsg;
	int FrontMsg;
	double Camera_HighMsg;
//================scan======================
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
	int Frame_Area(int coordinate, int range);
	int Angle_Adjustment(int angle);
	int Angle_Interval(int radius);
//================color====================
	int ColorModeMsg;
	vector<int> HSV_red;
	vector<int> HSV_green;
	vector<int> HSV_blue;
	vector<int> HSV_yellow;
	vector<int> HSV_white;
//================white===================
	int WhiteGrayMsg;
	int WhiteAngleMsg;
//================black===================
	int BlackGrayMsg;
	int BlackAngleMsg;
private:
	ros::NodeHandle nh;
	ros::Subscriber modebutton_sub;
	void modebuttoncall(const vision::parameterbutton msg);
	ros::Publisher interface_pub;
//===============save=====================
    ros::ServiceServer save_srv;
    bool savecall(std_srvs::Empty::Request  &req,
                  std_srvs::Empty::Response &res);
//==============camera====================
	ros::Subscriber camera_sub;
	void cameracall(const vision::camera msg);
	double camera_exposure;
//==============center====================
	ros::Subscriber center_sub;
	void centercall(const vision::center msg);
//==============distance==================
	ros::Subscriber position_sub;
	ros::Publisher distance_pub;
	void Pub_distance(int x, int y);
	void positioncall(const vision::position msg);
	int onclick;
	int mousex;
	int mousey;
	double camera_f(double Omni_pixel);
	double Omni_distance(double pixel_dis);
//===============scan====================
	ros::Subscriber scan_sub;
	void scancall(const vision::scan msg);
	void Set_Unscaned_Angle();
//===============color===================
	ros::Subscriber color_sub;
	void colorcall(const vision::color msg);
	void HSVmap();
	vector<BYTE> ColorFile();
//==============white====================
	ros::Subscriber white_sub;
	void whitecall(const vision::white msg);
//==============black====================
	ros::Subscriber black_sub;
	void blackcall(const vision::black msg);
};

