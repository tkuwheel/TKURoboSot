#include "nodehandle.h"

NodeHandle::NodeHandle()
{
    Readyaml();
    AngleLUT();
    buttonMsg = 1;
    ColorModeMsg = 0;
    save_srv =  nh.advertiseService("interface/save_srv", &NodeHandle::savecall, this);
    modebutton_sub = nh.subscribe("interface/parameterbutton", 1, &NodeHandle::modebuttoncall, this);
    camera_sub = nh.subscribe("interface/camera", 1, &NodeHandle::cameracall, this);
    center_sub = nh.subscribe("interface/center", 1, &NodeHandle::centercall, this);
    position_sub = nh.subscribe("interface/position", 1, &NodeHandle::positioncall, this);
    scan_sub = nh.subscribe("interface/scan", 1, &NodeHandle::scancall, this);
    color_sub = nh.subscribe("interface/color", 1, &NodeHandle::colorcall, this);
    white_sub = nh.subscribe("interface/white", 1, &NodeHandle::whitecall, this);
    black_sub = nh.subscribe("interface/black", 1, &NodeHandle::blackcall, this);
    distance_pub = nh.advertise<vision::dis>("interface/CenterDis", 1);
    interface_pub = nh.advertise<sensor_msgs::Image>("camera/image", 1);
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
    }
    else
    {
        cout << "yaml file does not exist" << endl;
    }
    Parameter_getting();
}
void NodeHandle::Saveyaml()
{
    std::string param = YAML_PATH;
    std::string temp = "rosparam dump " + param + " FIRA/vision";
    const char *save = temp.c_str();
    system(save);
    cout << "Save the yaml file" << endl;
    Parameter_getting();
}
void NodeHandle::Parameter_getting()
{
    cout << "get parameter" << endl;
    //===================FPS參數==========================
    nh.getParam("FIRA/vision/FPS", fpsMsg);
    //===================中心參數=========================
    nh.getParam("FIRA/vision/Center/Center_X", CenterXMsg);
    nh.getParam("FIRA/vision/Center/Center_Y", CenterYMsg);
    nh.getParam("FIRA/vision/Center/Inner", InnerMsg);
    nh.getParam("FIRA/vision/Center/Outer", OuterMsg);
    nh.getParam("FIRA/vision/Center/Front", FrontMsg);
    nh.getParam("FIRA/vision/Center/Camera_high", Camera_HighMsg);
    nh.getParam("FIRA/vision/Center/Horizon", HorizonMsg);
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
    nh.getParam("FIRA/vision/HSV/ColorMode", ColorModeMsg);
    nh.getParam("FIRA/vision/HSV/Ball", HSV_red);
    nh.getParam("FIRA/vision/HSV/Blue", HSV_blue);
    nh.getParam("FIRA/vision/HSV/Yellow", HSV_yellow);
    nh.getParam("FIRA/vision/HSV/Green", HSV_green);
    nh.getParam("FIRA/vision/HSV/White", HSV_white);
    nh.getParam("FIRA/vision/HSV/Redcone", HSV_redcone);
    //==================黑白掃描參數=======================
    nh.getParam("FIRA/vision/HSV/white/gray", WhiteGrayMsg);
    nh.getParam("FIRA/vision/HSV/white/angle", WhiteAngleMsg);
    nh.getParam("FIRA/vision/HSV/black/gray", BlackGrayMsg);
    nh.getParam("FIRA/vision/HSV/black/angle", BlackAngleMsg);
}

//======================前置處理結束=======================
//========================save===========================
bool NodeHandle::savecall(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
	//cout<<"Save\n";
    Saveyaml();
    HSVmap();
	return true;
}
//========================mode===========================
void NodeHandle::modebuttoncall(const vision::parameterbutton msg)
{
    buttonMsg = msg.button;
    std::cout << "mode: " << buttonMsg << std::endl;
}
//=======================camera==========================
void NodeHandle::cameracall(const vision::camera msg)
{
    fpsMsg = msg.fps;
    set_campara(fpsMsg);
}
void NodeHandle::get_campara()
{
    camera_exposure = 0.025;
    nh.getParam("prosilica_driver/exposure", camera_exposure);
}
void NodeHandle::set_campara(int value_ex)
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;
    double exposure = (double)value_ex / 1000;
    double_param.name = "exposure";
    double_param.value = exposure;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;
    ros::service::call("prosilica_driver/set_parameters", srv_req, srv_resp);
}

//========================center==========================
void NodeHandle::centercall(const vision::center msg)
{
    CenterXMsg = msg.CenterX;
    CenterYMsg = msg.CenterY;
    InnerMsg = msg.Inner;
    OuterMsg = msg.Outer;
    FrontMsg = msg.Front;
    Camera_HighMsg = msg.Camera_High;
    HorizonMsg = msg.Horizon;
}
//========================distance========================
void NodeHandle::positioncall(const vision::position msg)
{
    onclick = 1;
    mousex = msg.PositionX;
    mousey = msg.PositionY;
    Pub_distance(mousex, mousey);
}
/*void NodeHandle::onMouse(int Event, int x, int y, int flags, void* param)
{
	if (Event == CV_EVENT_LBUTTONDOWN) {
		mousex = x;
		mousey = y;
		cout<<"("<<x<<","<<y<<")"<<endl;
		onclick = 1;
	}	
}*/
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
void NodeHandle::Pub_distance(int x, int y)
{
    vision::dis dis_msg;
    double pixel_dis = sqrt(pow(mousex - CenterXMsg, 2) + pow(-1 * (mousey - CenterYMsg), 2));
    dis_msg.distance = Omni_distance(pixel_dis);
    distance_pub.publish(dis_msg);
    onclick = 0;
}
//===========================scan=================================
void NodeHandle::scancall(const vision::scan msg)
{
    Angle_Near_GapMsg = msg.Angle_Near_Gap;
    Magn_Near_GapMsg = msg.Magn_Near_Gap;
    Magn_Near_StartMsg = msg.Magn_Near_Start;
    Magn_Middle_StartMsg = msg.Magn_Middle_Start;
    Magn_Far_StartMsg = msg.Magn_Far_Start;
    Magn_Far_EndMsg = msg.Magn_Far_End;
    Dont_Search_Angle_1Msg = msg.Dont_Search_Angle_1;
    Dont_Search_Angle_2Msg = msg.Dont_Search_Angle_2;
    Dont_Search_Angle_3Msg = msg.Dont_Search_Angle_3;
    Angle_range_1Msg = msg.Angle_range_1;
    Angle_range_2_3Msg = msg.Angle_range_2_3;
    Set_Unscaned_Angle();
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
    if (angle_gap < 1)
    {
        angle_gap = 1;
    }
    return angle_gap;
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
//===========================color=================================
void NodeHandle::colorcall(const vision::color msg)
{
    ColorModeMsg = msg.ColorMode;
    switch (ColorModeMsg)
    {
    case 0:
        int BallHSVBoxMsg[6];
        for (int i = 0; i < 6; i++)
            BallHSVBoxMsg[i] = msg.BallHSVBox[i];
        HSV_red.assign(BallHSVBoxMsg, BallHSVBoxMsg + sizeof(BallHSVBoxMsg) / sizeof(int));
        break;
    case 1:
        int GreenHSVBoxMsg[6];
        for (int i = 0; i < 6; i++)
            GreenHSVBoxMsg[i] = msg.GreenHSVBox[i];
        HSV_green.assign(GreenHSVBoxMsg, GreenHSVBoxMsg + sizeof(GreenHSVBoxMsg) / sizeof(int));
        break;
    case 2:
        int BlueHSVBoxMsg[6];
        for (int i = 0; i < 6; i++)
            BlueHSVBoxMsg[i] = msg.BlueHSVBox[i];
        HSV_blue.assign(BlueHSVBoxMsg, BlueHSVBoxMsg + sizeof(BlueHSVBoxMsg) / sizeof(int));
        break;
    case 3:
        int YellowHSVBoxMsg[6];
        for (int i = 0; i < 6; i++)
            YellowHSVBoxMsg[i] = msg.YellowHSVBox[i];
        HSV_yellow.assign(YellowHSVBoxMsg, YellowHSVBoxMsg + sizeof(YellowHSVBoxMsg) / sizeof(int));
        break;
    case 4:
        int WhiteHSVBoxMsg[6];
        for (int i = 0; i < 6; i++)
            WhiteHSVBoxMsg[i] = msg.WhiteHSVBox[i];
        HSV_white.assign(WhiteHSVBoxMsg, WhiteHSVBoxMsg + sizeof(WhiteHSVBoxMsg) / sizeof(int));
        break;
    case 5:
        int RedconeHSVBoxMsg[6];
        for (int i = 0; i < 6; i++)
            RedconeHSVBoxMsg[i] = msg.RedconeHSVBox[i];
        HSV_redcone.assign(RedconeHSVBoxMsg, RedconeHSVBoxMsg + sizeof(RedconeHSVBoxMsg) / sizeof(int));
        break;
    }
}
void NodeHandle::HSVmap()
{
    unsigned char *HSVmap = new unsigned char[256 * 256 * 256];
    if (!HSV_red.empty() && !HSV_green.empty() && !HSV_blue.empty() && !HSV_yellow.empty() && !HSV_white.empty() && !HSV_redcone.empty())
    {
        //cout<<"get all hsv"<<endl;
        for (int b = 0; b < 256; b++)
        {
            for (int g = 0; g < 256; g++)
            {
                for (int r = 0; r < 256; r++)
                {
                    double R, G, B, H_sum, S_sum, V_sum;
                    B = b / 255.0;
                    G = g / 255.0;
                    R = r / 255.0;
                    double Max = (max(R, G) > max(G, B)) ? max(R, G) : max(G, B);
                    double Min = (min(R, G) < min(G, B)) ? min(R, G) : min(G, B);
                    if (Max == Min)
                    {
                        Max += 1;
                    }
                    if (R == Max)
                    {
                        H_sum = (G - B) * 60 / (Max - Min);
                    }
                    if (G == Max)
                    {
                        H_sum = 120 + (B - R) * 60 / (Max - Min);
                    }
                    if (B == Max)
                    {
                        H_sum = 240 + (R - G) * 60 / (Max - Min);
                    }
                    if (B == G && B == R)
                    {
                        H_sum = 0;
                    }
                    if (H_sum < 0)
                    {
                        H_sum = H_sum + 360;
                    }
                    if (Max == 0)
                    {
                        S_sum = 0;
                    }
                    S_sum = (((Max - Min) * 100) / Max);
                    V_sum = Max * 100;
                    HSVmap[r + (g << 8) + (b << 16)] = 0x00;
                    if (HSV_red[0] < HSV_red[1])
                    {
                        if ((H_sum >= HSV_red[0]) && (H_sum <= HSV_red[1]) && (S_sum >= HSV_red[2]) && (S_sum <= HSV_red[3]) && (V_sum >= HSV_red[4]) && (V_sum <= HSV_red[5]))
                            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | REDITEM;
                    }
                    else
                    {
                        if ((H_sum >= HSV_red[0]) || (H_sum <= HSV_red[1]) && (S_sum >= HSV_red[2]) && (S_sum <= HSV_red[3]) && (V_sum >= HSV_red[4]) && (V_sum <= HSV_red[5]))
                            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | REDITEM;
                    }
                    if (HSV_green[0] < HSV_green[1])
                    {
                        if ((H_sum >= HSV_green[0]) && (H_sum <= HSV_green[1]) && (S_sum >= HSV_green[2]) && (S_sum <= HSV_green[3]) && (V_sum >= HSV_green[4]) && (V_sum <= HSV_green[5]))
                            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | GREENITEM;
                    }
                    else
                    {
                        if ((H_sum >= HSV_green[0]) || (H_sum <= HSV_green[1]) && (S_sum >= HSV_green[2]) && (S_sum <= HSV_green[3]) && (V_sum >= HSV_green[4]) && (V_sum <= HSV_green[5]))
                            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | GREENITEM;
                    }
                    if (HSV_blue[0] < HSV_blue[1])
                    {

                        if ((H_sum >= HSV_blue[0]) && (H_sum <= HSV_blue[1]) && (S_sum >= HSV_blue[2]) && (S_sum <= HSV_blue[3]) && (V_sum >= HSV_blue[4]) && (V_sum <= HSV_blue[5]))
                            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | BLUEITEM;
                    }
                    else
                    {
                        if ((H_sum >= HSV_blue[0]) || (H_sum <= HSV_blue[1]) && (S_sum >= HSV_blue[2]) && (S_sum <= HSV_blue[3]) && (V_sum >= HSV_blue[4]) && (V_sum <= HSV_blue[5]))
                            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | BLUEITEM;
                    }
                    if (HSV_yellow[0] < HSV_yellow[1])
                    {
                        if ((H_sum >= HSV_yellow[0]) && (H_sum <= HSV_yellow[1]) && (S_sum >= HSV_yellow[2]) && (S_sum <= HSV_yellow[3]) && (V_sum >= HSV_yellow[4]) && (V_sum <= HSV_yellow[5]))
                            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | YELLOWITEM;
                    }
                    else
                    {
                        if ((H_sum >= HSV_yellow[0]) || (H_sum <= HSV_yellow[1]) && (S_sum >= HSV_yellow[2]) && (S_sum <= HSV_yellow[3]) && (V_sum >= HSV_yellow[4]) && (V_sum <= HSV_yellow[5]))
                            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | YELLOWITEM;
                    }
                    if (HSV_white[0] < HSV_white[1])
                    {
                        if ((H_sum >= HSV_white[0]) && (H_sum <= HSV_white[1]) && (S_sum >= HSV_white[2]) && (S_sum <= HSV_white[3]) && (V_sum >= HSV_white[4]) && (V_sum <= HSV_white[5]))
                            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | WHITEITEM;
                    }
                    else
                    {
                        if ((H_sum >= HSV_white[0]) || (H_sum <= HSV_white[1]) && (S_sum >= HSV_white[2]) && (S_sum <= HSV_white[3]) && (V_sum >= HSV_white[4]) && (V_sum <= HSV_white[5]))
                            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | WHITEITEM;
                    }
                    if (HSV_redcone[0] < HSV_redcone[1])
                    {
                        if ((H_sum >= HSV_redcone[0]) && (H_sum <= HSV_redcone[1]) && (S_sum >= HSV_redcone[2]) && (S_sum <= HSV_redcone[3]) && (V_sum >= HSV_redcone[4]) && (V_sum <= HSV_redcone[5]))
                            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | REDCONEITEM;
                    }
                    else
                    {
                        if ((H_sum >= HSV_redcone[0]) || (H_sum <= HSV_redcone[1]) && (S_sum >= HSV_redcone[2]) && (S_sum <= HSV_redcone[3]) && (V_sum >= HSV_redcone[4]) && (V_sum <= HSV_redcone[5]))
                            HSVmap[r + (g << 8) + (b << 16)] = HSVmap[r + (g << 8) + (b << 16)] | REDCONEITEM;
                    }
                }
            }
        }
        //#define BIN_PATH ros::package::getPath("vision")+"/config/HSVcolormap.bin"
        cout << "save the bin file" << endl;
        string Filename = BIN_PATH;
        const char *Filename_Path = Filename.c_str();
        FILE *file = fopen(Filename_Path, "rb+"); //開啟檔案來寫
        fwrite(HSVmap, 1, 256 * 256 * 256, file);
        fclose(file);
    }
}
//==============================white=========================
void NodeHandle::whitecall(const vision::white msg)
{
    WhiteGrayMsg = msg.Gray;
    WhiteAngleMsg = msg.Angle;
}
//==============================black=========================
void NodeHandle::blackcall(const vision::black msg)
{
    BlackGrayMsg = msg.Gray;
    BlackAngleMsg = msg.Angle;
}
//=============================publisher======================
void NodeHandle::Pub_interface(Mat frame)
{
    if (buttonMsg >= 0 && buttonMsg <= 6)
    {
        sensor_msgs::ImagePtr interfaceMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        interface_pub.publish(interfaceMsg);
    }
}
