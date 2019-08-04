#include <cstdio>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <highgui.h>
#include <vector>
#include <deque>
#include <math.h>
#include <signal.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <fstream>
#include "vision/Two_point.h"
#include "vision/Object.h"
#include "vision/bin.h"
#include "vision/view.h"

#define PI 3.14159265
#define FRAME_COLS 659 //width  x695
#define FRAME_ROWS 493 //height y493
#define REDITEM 0x01
#define GREENITEM 0x02
#define BLUEITEM 0x04
#define YELLOWITEM 0x08
#define WHITEITEM 0x10 //WHITEITEM=robot
#define VISION_TOPIC "camera/image_raw"
#define USB_CAM_TOPIC "usb_cam/image_raw"
#define YAML_PATH ros::package::getPath("vision") + "/config/FIRA.yaml"
#define BIN_PATH ros::package::getPath("vision") + "/config/HSVcolormap.bin"
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

    int left_dis;     //pix
    int right_dis;    //pix
    int left_x;       //pix
    int left_y;       //pix
    int right_x;      //pix
    int right_y;      //pix
    int fix_x;        //pix
    int fix_y;        //pix
    int fix_angle;    //pix
    int fix_distance; //pix
    int fix_ang_max;  //pix
    int fix_ang_min;  //pix
    string LR;
};

class NodeHandle
{
  protected:
    NodeHandle();
    void Readyaml();
    void Parameter_getting();

    vector<double> Angle_sin;
    vector<double> Angle_cos;
    int SizeFilter;
    //======================================
    void Set_Unscaned_Angle();
    void HSVmap();
    vector<BYTE> ColorFile();
    vector<BYTE> color_map;
    int b_end_gap;
    int y_end_gap;
    //======================================
    void AngleLUT();
    int Strategy_Angle(int angle);
    int Frame_Area(int coordinate, int range);
    int Angle_Adjustment(int angle);
    double camera_f(double Omni_pixel);
    double Omni_distance(double pixel_dis);
    int Angle_Interval(int radius);
    double RateMsg;
    DetectedObject Red_Item, Blue_Item, Yellow_Item;
    //======================================
    //===============publisher==============
    void Pub_monitor(Mat Monitor);
    void Pub_object();
    void Pub_goal_edge();
    //==============subscriber=============
    vector<int> HSV_red;
    vector<int> HSV_green;
    vector<int> HSV_blue;
    vector<int> HSV_yellow;
    vector<int> HSV_white;
    int CenterXMsg;
    int CenterYMsg;
    int InnerMsg;
    int OuterMsg;
    int FrontMsg;
    double Camera_HighMsg;
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
    //================white===================
    int WhiteGrayMsg;
    int WhiteAngleMsg;
    //================black===================
    int BlackGrayMsg;
    int BlackAngleMsg;
    //========================================
  private:
    ros::NodeHandle nh;
    ros::Subscriber save_sub;
    void SaveButton_setting(const vision::bin msg);
    int SaveButton;
    //ros::Subscriber view_sub;
    //void View(const vision::view msg);
    //int viewcheck;
    ros::Publisher monitor_pub;
    ros::Publisher object_pub;
    ros::Publisher Two_point_pub;
    //====================================
    ros::ServiceServer connect_srv;
    bool connectcall(std_srvs::Empty::Request  &req,
                 std_srvs::Empty::Response &res);
};
