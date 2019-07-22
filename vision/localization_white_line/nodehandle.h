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
#include "vision/white.h"
#include "vision/black.h"
#include "vision/bin.h"
#include "vision/color.h"
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
    NodeHandle();
    void Readyaml();
    void Parameter_getting();
    //==========================================
    void AngleLUT();
    vector<double> Angle_sin;
    vector<double> Angle_cos;
    int Frame_Area(int coordinate, int range);
    int Angle_Adjustment(int angle);
    void Pub_whiteframe(Mat frame);
    void Pub_whitedis(std_msgs::Int32MultiArray distance);
    //================center====================
    int CenterXMsg;
    int CenterYMsg;
    int InnerMsg;
    int OuterMsg;
    int FrontMsg;
    double Camera_HighMsg;
    //================white===================
    int WhiteGrayMsg;
    int WhiteAngleMsg;
    std_msgs::Int32MultiArray whitedis;
    //==============distance==================
    double camera_f(double Omni_pixel);
    double Omni_distance(double pixel_dis);
    //========================================
    vector<int> HSV_green;

  private:
    ros::NodeHandle nh;
    ros::Subscriber save_sub;
    void SaveButton_setting(const vision::bin msg);
    int SaveButton;
    //==============white====================
    ros::Publisher whiteframe_pub;
    ros::Publisher whitedis_pub;
    void whitecall(const vision::white msg);
    //=======================================
    ros::Subscriber color_sub;
	void colorcall(const vision::color msg);
};
