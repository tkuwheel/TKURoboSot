#include "nodehandle.h"

DetectedObject::DetectedObject()
{
    dis_max = 0;
    dis_min = 0;
    ang_max = 0;
    ang_min = 0;
    x = 0;
    y = 0;
    angle = 0;
    distance = 0;
    size = 0;
}
void DetectedObject::Reset()
{
    dis_max = 0;
    dis_min = 0;
    ang_max = 0;
    ang_min = 0;
    x = 0;
    y = 0;
    angle = 0;
    distance = 0;
    size = 0;
}
NodeHandle::NodeHandle()
{
    Readyaml();
    AngleLUT();

    save_sub = nh.subscribe("interface/bin_save", 1000, &NodeHandle::SaveButton_setting, this);
    //http://localhost:8080/stream?topic=/camera/image_monitor webfor /camera/image
    obstacle_pub = nh.advertise<std_msgs::Int32MultiArray>("vision/obstacle", 1);
    pixel_obstacle_pub = nh.advertise<std_msgs::Int32MultiArray>("vision/pixel_obstacle", 1);
    obstacleframe_pub = nh.advertise<sensor_msgs::Image>("camera/obstacle", 1);
    
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
    nh.getParam("FIRA/vision/Center/Horizon", HorizonMsg);
    //==================黑白掃描參數=======================
    nh.getParam("FIRA/vision/HSV/black/gray", BlackGrayMsg);
    nh.getParam("FIRA/vision/HSV/black/angle", BlackAngleMsg);
    //==================掃瞄點參數=========================
    nh.getParam("FIRA/vision/SCAN/Angle_Near_Gap", Angle_Near_GapMsg);
    nh.getParam("FIRA/vision/SCAN/Magn_Near_Gap", Magn_Near_GapMsg);
    nh.getParam("FIRA/vision/SCAN/Magn_Near_Start", Magn_Near_StartMsg);
    nh.getParam("FIRA/vision/SCAN/Magn_Middle_Start", Magn_Middle_StartMsg);
    nh.getParam("FIRA/vision/SCAN/Magn_Far_Start", Magn_Far_StartMsg);
    nh.getParam("FIRA/vision/SCAN/Magn_Far_End", Magn_Far_EndMsg);
    nh.getParam("FIRA/vision/SCAN/Dont_Search_Angle_1", Dont_Search_Angle_1Msg);
    nh.getParam("FIRA/vision/SCAN/Dont_Search_Angle_2", Dont_Search_Angle_2Msg);
    nh.getParam("FIRA/vision/SCAN/Dont_Search_Angle_3", Dont_Search_Angle_3Msg);
    nh.getParam("FIRA/vision/SCAN/Angle_range_1", Angle_range_1Msg);
    nh.getParam("FIRA/vision/SCAN/Angle_range_2_3", Angle_range_2_3Msg);
    Set_Unscaned_Angle();
    //====================================================
    nh.getParam("FIRA/vision/HSV/White", HSV_robot);
}
void NodeHandle::Set_Unscaned_Angle()
{
    Unscaned_Angle[0] = Angle_Adjustment(Dont_Search_Angle_1Msg - Angle_range_1Msg);
    Unscaned_Angle[1] = Angle_Adjustment(Dont_Search_Angle_1Msg + Angle_range_1Msg);
    Unscaned_Angle[2] = Angle_Adjustment(Dont_Search_Angle_2Msg - Angle_range_2_3Msg);
    Unscaned_Angle[3] = Angle_Adjustment(Dont_Search_Angle_2Msg + Angle_range_2_3Msg);
    Unscaned_Angle[4] = Angle_Adjustment(Dont_Search_Angle_3Msg - Angle_range_2_3Msg);
    Unscaned_Angle[5] = Angle_Adjustment(Dont_Search_Angle_3Msg + Angle_range_2_3Msg);
    Unscaned_Angle[6] = 999;
    Unscaned_Angle[7] = 999;

    if ((Dont_Search_Angle_1Msg - Angle_range_1Msg) <= 0 || (Dont_Search_Angle_1Msg + Angle_range_1Msg) >= 360)
    {
        Unscaned_Angle[0] = 0;
        Unscaned_Angle[1] = Angle_Adjustment(Dont_Search_Angle_1Msg + Angle_range_1Msg);
        Unscaned_Angle[2] = Angle_Adjustment(Dont_Search_Angle_2Msg - Angle_range_2_3Msg);
        Unscaned_Angle[3] = Angle_Adjustment(Dont_Search_Angle_2Msg + Angle_range_2_3Msg);
        Unscaned_Angle[4] = Angle_Adjustment(Dont_Search_Angle_3Msg - Angle_range_2_3Msg);
        Unscaned_Angle[5] = Angle_Adjustment(Dont_Search_Angle_3Msg + Angle_range_2_3Msg);
        Unscaned_Angle[6] = Angle_Adjustment(Dont_Search_Angle_1Msg - Angle_range_1Msg);
        Unscaned_Angle[7] = 360;
    }

    if ((Dont_Search_Angle_2Msg - Angle_range_2_3Msg) <= 0 || (Dont_Search_Angle_2Msg + Angle_range_2_3Msg) >= 360)
    {
        Unscaned_Angle[0] = Angle_Adjustment(Dont_Search_Angle_1Msg - Angle_range_1Msg);
        Unscaned_Angle[1] = Angle_Adjustment(Dont_Search_Angle_1Msg + Angle_range_1Msg);
        Unscaned_Angle[2] = 0;
        Unscaned_Angle[3] = Angle_Adjustment(Dont_Search_Angle_2Msg + Angle_range_2_3Msg);
        Unscaned_Angle[4] = Angle_Adjustment(Dont_Search_Angle_3Msg - Angle_range_2_3Msg);
        Unscaned_Angle[5] = Angle_Adjustment(Dont_Search_Angle_3Msg + Angle_range_2_3Msg);
        Unscaned_Angle[6] = Angle_Adjustment(Dont_Search_Angle_2Msg - Angle_range_2_3Msg);
        Unscaned_Angle[7] = 360;
    }

    if ((Dont_Search_Angle_3Msg - Angle_range_2_3Msg) <= 0 || (Dont_Search_Angle_3Msg + Angle_range_2_3Msg) >= 360)
    {
        Unscaned_Angle[0] = Angle_Adjustment(Dont_Search_Angle_1Msg - Angle_range_1Msg);
        Unscaned_Angle[1] = Angle_Adjustment(Dont_Search_Angle_1Msg + Angle_range_1Msg);
        Unscaned_Angle[2] = Angle_Adjustment(Dont_Search_Angle_2Msg - Angle_range_2_3Msg);
        Unscaned_Angle[3] = Angle_Adjustment(Dont_Search_Angle_2Msg + Angle_range_2_3Msg);
        Unscaned_Angle[4] = 0;
        Unscaned_Angle[5] = Angle_Adjustment(Dont_Search_Angle_3Msg + Angle_range_2_3Msg);
        Unscaned_Angle[6] = Angle_Adjustment(Dont_Search_Angle_3Msg - Angle_range_2_3Msg);
        Unscaned_Angle[7] = 360;
    }
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
//角度間隔
//middle start 到 far start 之間　Angle near gap的值為1/2
//far start 之外 Angle near gap的值為1/4
int NodeHandle::Angle_Interval(int radius)
{
    int angle_gap = 1;
    if (radius <= Magn_Middle_StartMsg)
    {
        angle_gap = Angle_Near_GapMsg;
    }
    else if (radius > Magn_Middle_StartMsg && radius <= Magn_Far_StartMsg)
    {
        angle_gap = Angle_Near_GapMsg / 2;
    }
    else if (radius > Magn_Far_StartMsg && radius <= Magn_Far_EndMsg)
    {
        angle_gap = Angle_Near_GapMsg / 4;
    }
    if (angle_gap <= 0)
    {
        angle_gap = 1;
    }
    return angle_gap;
}
//========================save============================
void NodeHandle::SaveButton_setting(const vision::bin msg)
{
    //cout<<"Save\n";
    SaveButton = msg.bin;
    Parameter_getting();
}
//========================distance========================
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
int NodeHandle::Strategy_Angle(int angle)
{
    if (Angle_Adjustment(angle - FrontMsg) < 180)
    {
        angle = Angle_Adjustment(angle - FrontMsg);
    }
    else
    {
        angle = Angle_Adjustment(angle - FrontMsg) - 360;
    }
    return angle;
}
//========================publisher=======================
void NodeHandle::pub_obstacle(vector<DetectedObject> obstacle)
{
    std_msgs::Int32MultiArray obstacle_info;
    for(int i=0; i<obstacle.size(); i++){
        obstacle_info.data.push_back(Omni_distance(obstacle.at(i).distance));
        obstacle_info.data.push_back(Strategy_Angle(obstacle.at(i).angle));
        obstacle_info.data.push_back(Strategy_Angle(obstacle.at(i).ang_max));
        obstacle_info.data.push_back(Strategy_Angle(obstacle.at(i).ang_min));
    }
    obstacle_pub.publish(obstacle_info);
}
void NodeHandle::pub_pixel_obstacle(vector<DetectedObject> obstacle)
{
    std_msgs::Int32MultiArray obstacle_info;
    for(int i=0; i<obstacle.size(); i++){
        obstacle_info.data.push_back(obstacle.at(i).distance);
        obstacle_info.data.push_back(obstacle.at(i).angle);
        obstacle_info.data.push_back(obstacle.at(i).ang_max);
        obstacle_info.data.push_back(obstacle.at(i).ang_min);
    }
    pixel_obstacle_pub.publish(obstacle_info);
}
void NodeHandle::Pub_obstacleframe(Mat frame)
{
    sensor_msgs::ImagePtr obstacleframeMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    obstacleframe_pub.publish(obstacleframeMsg);
}
