#include "nodehandle.h"

NodeHandle::NodeHandle()
{
    Readyaml();
    AngleLUT();

    save_sub = nh.subscribe("interface/bin_save", 1000, &NodeHandle::SaveButton_setting, this);
    //blackframe_pub = nh.advertise<sensor_msgs::Image>("camera/black", 1);
    blackdis_pub = nh.advertise<std_msgs::Int32MultiArray>("vision/BlackRealDis", 1);
    //http://localhost:8080/stream?topic=/camera/image_monitor webfor /camera/image
}
void NodeHandle::AngleLUT()
{
    double ang_PI;
    for (int ang = 0; ang <= 360; ang++)
    {
        ang_PI = ang * PI / 180;
        Angle_sin.push_back(sin(ang_PI));
        Angle_cos.push_back(cos(ang_PI));
    }
}
void NodeHandle::Readyaml()
{
    std::string param = YAML_PATH;
    const char *parampath = param.c_str();
    if (ifstream(parampath))
    {
        std::string temp = "rosparam load " + param + " FIRA/vision";
        const char *load = temp.c_str();
        system(load);
        cout << "Read the yaml file" << endl;
        Parameter_getting();
    }
    else
    {
        ROS_ERROR("yaml file does not exist");
    }
}
void NodeHandle::Parameter_getting()
{
    cout << "get parameter" << endl;
    //===================中心參數=========================
    nh.getParam("FIRA/vision/Center/Center_X", CenterXMsg);
    nh.getParam("FIRA/vision/Center/Center_Y", CenterYMsg);
    nh.getParam("FIRA/vision/Center/Inner", InnerMsg);
    nh.getParam("FIRA/vision/Center/Outer", OuterMsg);
    nh.getParam("FIRA/vision/Center/Front", FrontMsg);
    nh.getParam("FIRA/vision/Center/Camera_high", Camera_HighMsg);
    //==================黑白掃描參數=======================
    nh.getParam("FIRA/vision/HSV/black/gray", BlackGrayMsg);
    nh.getParam("FIRA/vision/HSV/black/angle", BlackAngleMsg);
}
//======================前置處理結束========================
int NodeHandle::Frame_Area(int coordinate, int range)
{
    if (coordinate < 0)
        coordinate = 0;
    else if (coordinate >= range)
        coordinate = range - 1;
    return coordinate;
}
//角度調整
//修正大於或小於360的角度
int NodeHandle::Angle_Adjustment(int angle)
{
    if (angle < 0)
        return angle + 360;
    else if (angle >= 360)
        return angle - 360;
    else
        return angle;
}
//========================save============================
void NodeHandle::SaveButton_setting(const vision::bin msg)
{
    //cout<<"Save\n";
    SaveButton = msg.bin;
    Parameter_getting();
}
//========================distance========================
//========================publisher=======================
double NodeHandle::camera_f(double Omni_pixel)
{
    double m = (Omni_pixel * 0.0099) / 60; // m = H1/H0 = D1/D0    D0 + D1 = 180
    double D0 = 180 / (1 + m);             // D1 = m   *D0
    double D1 = 180 / (1 + (1 / m));       // D0 = 1/m *D1
    double f = 1 / (1 / D0 + 1 / D1);
    //ROS_INFO("m = %f D0 = %f D1 = %f F = %f",m,D0,D1,f);
    return D1;
}
double NodeHandle::Omni_distance(double pixel_dis)
{
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
//========================publisher=======================
void NodeHandle::Pub_blackframe(Mat frame)
{
    sensor_msgs::ImagePtr blackframeMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    blackframe_pub.publish(blackframeMsg);
}
void NodeHandle::Pub_blackdis(std_msgs::Int32MultiArray distance)
{
    blackdis_pub.publish(distance);
}
