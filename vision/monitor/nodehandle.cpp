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
    LR = "Null";

    left_dis = 999;
    right_dis = 999;
    left_x = 0;
    left_y = 0;
    right_x = 0;
    right_y = 0;
    fix_x = 0;
    fix_y = 0;
    fix_angle = 0;
    fix_distance = 0;
    fix_ang_max = 0;
    fix_ang_min = 0;
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
    LR = "Null";

    left_dis = 999;
    right_dis = 999;
    left_x = 0;
    left_y = 0;
    right_x = 0;
    right_y = 0;
    fix_x = 0;
    fix_y = 0;
    fix_angle = 0;
    fix_distance = 0;
    fix_ang_max = 0;
    fix_ang_min = 0;
}
NodeHandle::NodeHandle()
{
    //Parameter_default();
    Readyaml();
    AngleLUT();
    SizeFilter = 5;
    connect_srv = nh.advertiseService("monitor/connect", &NodeHandle::connectcall, this);
    save_sub = nh.subscribe("interface/bin_save", 1000, &NodeHandle::SaveButton_setting, this);
    //view_sub = nh.subscribe("vision/view", 1000, &NodeHandle::View, this);
    //http://localhost:8080/stream?topic=/camera/image_monitor webfor /camera/image
    monitor_pub = nh.advertise<sensor_msgs::Image>("camera/image_monitor", 1);
    object_pub = nh.advertise<vision::Object>("vision/object", 1);
    Two_point_pub = nh.advertise<vision::Two_point>("interface/Two_point", 1);
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
    }
    else
    {
        cout << "YAML file does not exist" << endl;
        //exit(0);
    }
    Parameter_getting();
}
bool NodeHandle::connectcall(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
	//cout<<"program connect\n";
	return true;
}
void NodeHandle::SaveButton_setting(const vision::bin msg)
{
    //cout<<"Save\n";
    SaveButton = msg.bin;
    Parameter_getting();
}
//void NodeHandle::View(const vision::view msg)
//{
//	viewcheck = msg.checkpoint;
//}
void NodeHandle::Parameter_getting()
{

    if (nh.getParam("FIRA/vision/HSV/Ball", HSV_red))
    {
        cout << "get_parameter" << endl;
    }
    else
    {
        cout << "Unable to read parameters" << endl;
    }
    //===================中心參數=========================
    nh.getParam("FIRA/vision/Center/Center_X", CenterXMsg);
    nh.getParam("FIRA/vision/Center/Center_Y", CenterYMsg);
    nh.getParam("FIRA/vision/Center/Inner", InnerMsg);
    nh.getParam("FIRA/vision/Center/Outer", OuterMsg);
    nh.getParam("FIRA/vision/Center/Front", FrontMsg);
    nh.getParam("FIRA/vision/Center/Camera_high", Camera_HighMsg);
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
    //===================色模參數=========================
    nh.getParam("FIRA/vision/HSV/Ball", HSV_red);
    nh.getParam("FIRA/vision/HSV/Blue", HSV_blue);
    nh.getParam("FIRA/vision/HSV/Yellow", HSV_yellow);
    nh.getParam("FIRA/vision/HSV/Green", HSV_green);
    nh.getParam("FIRA/vision/HSV/White", HSV_white);
    //==================黑白掃描參數=======================
    nh.getParam("FIRA/vision/HSV/white/gray", WhiteGrayMsg);
    nh.getParam("FIRA/vision/HSV/white/angle", WhiteAngleMsg);
    nh.getParam("FIRA/vision/HSV/black/gray", BlackGrayMsg);
    nh.getParam("FIRA/vision/HSV/black/angle", BlackAngleMsg);
    color_map = ColorFile();
}
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
    double Z = -1 * Camera_HighMsg; //Camera_HighMsg=65;
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
//right -0~-180
//left   0~180
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
//掃描點座標調整
//修正超出圖片的點座標
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
vector<BYTE> NodeHandle::ColorFile()
{
    string vision_path = ros::package::getPath("vision");
    string FILE_PATH = "/config/HSVcolormap.bin";
    string Filename = vision_path + FILE_PATH;
    const char *Filename_Path = Filename.c_str();
    if (ifstream(Filename_Path)){
    // open the file:
    streampos fileSize;
    std::ifstream file(Filename_Path, ios::binary);
    // get its size:
    file.seekg(0, ios::end);
    fileSize = file.tellg();
    file.seekg(0, ios::beg);
    // read the data:
    vector<BYTE> fileData(fileSize);
    file.read((char *)&fileData[0], fileSize);
    return fileData;
    }
    else{
        cout << "COLORMAP file does not exist" << endl;
        exit(0);
    }
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
//===================================前置處理結束===================
void NodeHandle::Pub_monitor(Mat Monitor)
{
    sensor_msgs::ImagePtr monitormsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Monitor).toImageMsg();
    monitor_pub.publish(monitormsg);
}
void NodeHandle::Pub_object()
{
    vision::Object object_msg;
    //===================default===============
    object_msg.fps = 999;
    object_msg.ball_x = 999;
    object_msg.ball_y = 999;
    object_msg.ball_LR = "Null";
    object_msg.ball_ang = 999;
    object_msg.ball_dis = 999;

    object_msg.blue_x = 999;
    object_msg.blue_y = 999;
    object_msg.blue_LR = "Null";
    object_msg.blue_ang = 999;
    object_msg.blue_dis = 999;
    object_msg.blue_fix_x = 999;
    object_msg.blue_fix_y = 999;
    object_msg.blue_fix_ang = 999;
    object_msg.blue_fix_dis = 999;

    object_msg.yellow_x = 999;
    object_msg.yellow_y = 999;
    object_msg.yellow_LR = "Null";
    object_msg.yellow_ang = 999;
    object_msg.yellow_dis = 999;
    object_msg.yellow_fix_x = 999;
    object_msg.yellow_fix_y = 999;
    object_msg.yellow_fix_ang = 999;
    object_msg.yellow_fix_dis = 999;
    //==========================================
    object_msg.fps = RateMsg;

    // if (Red_Item.size > SizeFilter)
    {
        object_msg.ball_x = Red_Item.x - CenterXMsg;
        object_msg.ball_y = 0 - (Red_Item.y - CenterYMsg);
        object_msg.ball_LR = Red_Item.LR;
        object_msg.ball_ang = Strategy_Angle(Angle_Adjustment(Red_Item.angle));
        object_msg.ball_dis = Omni_distance(Red_Item.distance);
    }

    // if (Blue_Item.size > SizeFilter)
    {
        object_msg.blue_x = Blue_Item.x - CenterXMsg;
        object_msg.blue_y = 0 - (Blue_Item.y - CenterYMsg);
        object_msg.blue_LR = Blue_Item.LR;
        object_msg.blue_ang = Strategy_Angle(Angle_Adjustment(Blue_Item.angle));
        object_msg.blue_dis = Omni_distance(Blue_Item.distance);
        object_msg.blue_fix_x = Blue_Item.fix_x - CenterXMsg;
        object_msg.blue_fix_y = 0 - (Blue_Item.fix_y - CenterYMsg);
        object_msg.blue_fix_ang = Strategy_Angle(Angle_Adjustment(Blue_Item.fix_angle));
        object_msg.blue_fix_dis = Omni_distance(Blue_Item.fix_distance);
    }

    // if (Yellow_Item.size > SizeFilter)
    {
        object_msg.yellow_x = Yellow_Item.x - CenterXMsg;
        object_msg.yellow_y = 0 - (Yellow_Item.y - CenterYMsg);
        object_msg.yellow_LR = Yellow_Item.LR;
        object_msg.yellow_ang = Strategy_Angle(Angle_Adjustment(Yellow_Item.angle));
        object_msg.yellow_dis = Omni_distance(Yellow_Item.distance);
        object_msg.yellow_fix_x = Yellow_Item.fix_x - CenterXMsg;
        object_msg.yellow_fix_y = 0 - (Yellow_Item.fix_y - CenterYMsg);
        object_msg.yellow_fix_ang = Strategy_Angle(Angle_Adjustment(Yellow_Item.fix_angle));
        object_msg.yellow_fix_dis = Omni_distance(Yellow_Item.fix_distance);
    }
    object_pub.publish(object_msg);
}
void NodeHandle::Pub_goal_edge()
{
    vision::Two_point Two_point_msg;

    if (Blue_Item.size > SizeFilter)
    {
        Two_point_msg.blue_dis = Omni_distance(Blue_Item.distance);
        Two_point_msg.blue_ang_max = Strategy_Angle(Angle_Adjustment(Blue_Item.ang_max));
        Two_point_msg.blue_ang_min = Strategy_Angle(Angle_Adjustment(Blue_Item.ang_min));
        Two_point_msg.blue_fix_ang_max = Strategy_Angle(Angle_Adjustment(Blue_Item.fix_ang_max));
        Two_point_msg.blue_fix_ang_min = Strategy_Angle(Angle_Adjustment(Blue_Item.fix_ang_min));
        Two_point_msg.blue_left = Omni_distance(Blue_Item.left_dis);
        Two_point_msg.blue_right = Omni_distance(Blue_Item.right_dis);
    }
    if (Yellow_Item.size > SizeFilter)
    {
        Two_point_msg.yellow_dis = Omni_distance(Yellow_Item.distance);
        Two_point_msg.yellow_ang_max = Strategy_Angle(Angle_Adjustment(Yellow_Item.ang_max));
        Two_point_msg.yellow_ang_min = Strategy_Angle(Angle_Adjustment(Yellow_Item.ang_min));
        Two_point_msg.yellow_fix_ang_max = Strategy_Angle(Angle_Adjustment(Yellow_Item.fix_ang_max));
        Two_point_msg.yellow_fix_ang_min = Strategy_Angle(Angle_Adjustment(Yellow_Item.fix_ang_min));
        Two_point_msg.yellow_left = Omni_distance(Yellow_Item.left_dis);
        Two_point_msg.yellow_right = Omni_distance(Yellow_Item.right_dis);
    }
    Two_point_pub.publish(Two_point_msg);
}
