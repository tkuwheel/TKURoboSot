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

class NodeHandle
{
  protected:
	void Readyaml();
	NodeHandle();
	void Parameter_getting();
	//==========================================
	void AngleLUT();
	vector<double> Angle_sin;
	vector<double> Angle_cos;
	int Frame_Area(int coordinate, int range);
	int Angle_Adjustment(int angle);
	void Pub_blackframe(Mat frame);
	void Pub_blackdis(std_msgs::Int32MultiArray distance);
	//================center====================
	int CenterXMsg;
	int CenterYMsg;
	int InnerMsg;
	int OuterMsg;
	int FrontMsg;
	double Camera_HighMsg;
	//================black===================
	int BlackGrayMsg;
	int BlackAngleMsg;
	std_msgs::Int32MultiArray blackdis;
	//==============distance==================
	double camera_f(double Omni_pixel);
	double Omni_distance(double pixel_dis);

  private:
	ros::NodeHandle nh;
	ros::Subscriber save_sub;
	void SaveButton_setting(const vision::bin msg);
	int SaveButton;
	//==============black====================
	ros::Publisher blackframe_pub;
	ros::Publisher blackdis_pub;
	void blackcall(const vision::black msg);
};
