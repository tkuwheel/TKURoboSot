#include "interface.h"

Vision::Vision(string topic)
{
    image_sub = nh.subscribe(topic, 1, &Vision::imageCb, this);
    FrameRate = 0.0;
}
Vision::~Vision()
{
    Source.release();
    destroyAllWindows();
}
double Vision::Rate()
{
    double ALPHA = 0.5;
    double dt;
    static int frame_counter = 0;
    static double frame_rate = 0.0;
    static double StartTime = ros::Time::now().toNSec();
    double EndTime;

    frame_counter++;
    if (frame_counter == 10)
    {
        EndTime = ros::Time::now().toNSec();
        dt = (EndTime - StartTime) / frame_counter;
        StartTime = EndTime;
        if (dt != 0)
        {
            frame_rate = (1000000000.0 / dt) * ALPHA + frame_rate * (1.0 - ALPHA);
            //cout << "FPS: " << frame_rate << endl;
        }

        frame_counter = 0;
    }
    return frame_rate;
}
//int mousex,mousey;
//void onMouse(int Event, int x, int y, int flags, void* param)
//{
//	if (Event == CV_EVENT_LBUTTONDOWN) {
//		mousex = x;
//		mousey = y;
//		cout<<"("<<x<<","<<y<<")"<<endl;
//		onclick = 1;
//	}
//}
//=============================影像接收=======================================
void Vision::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    //void Vision::imageCb(const sensor_msgs::CompressedImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //convert image data
        Source = cv_ptr->image.clone();
        //resize(Source,Source,Size(Source.cols/2,Source.rows/2),0,0,INTER_LINEAR);
        //if(!cv_ptr->image.empty()){
        //	cv::imshow("Image window", cv_ptr->image);
        //	setMouseCallback("Image window", onMouse, NULL);
        //	cv::waitKey(10);
        //}
        //FrameRate = Rate();

        Mat outputframe = Source.clone();
        cv::flip(outputframe, outputframe, 1); // reverse image

        int visionmode = buttonMsg;
        switch (visionmode)
        {
        case 1:
            outputframe = CameraModel(outputframe);
            //cv::imshow("Image window", outputframe);
            //waitKey(3);
            break;
        case 2:
            outputframe = CenterModel(outputframe);
            //cv::imshow("Image window", outputframe);
            //waitKey(3);
            break;
        case 3:
            outputframe = ScanModel(outputframe);
            //cv::imshow("Image window", outputframe);
            //waitKey(3);
            break;
        case 4:
            outputframe = ColorModel(outputframe);
            //cv::imshow("Image window", outputframe);
            //waitKey(3);
            break;
        case 5:
            outputframe = White_Line(outputframe);
            //cv::imshow("Image window", outputframe);
            //waitKey(3);
            break;
        case 6:
            outputframe = Black_Line(outputframe);
            //cv::imshow("Image window", outputframe);
            //waitKey(3);
            break;
        }
        Pub_interface(outputframe);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert to image!");
        return;
    }
}
cv::Mat Vision::CameraModel(const cv::Mat iframe)
{
    //if(0<fpsMsg<=100){}else{fpsMsg=25;}
    //get_campara();
    //cout<<camera_exposure<<endl;
    set_campara(fpsMsg);
    return iframe;
}
cv::Mat Vision::CenterModel(const cv::Mat iframe)
{
    cv::Mat oframe = iframe.clone();
    Point center = Point(CenterXMsg, CenterYMsg);
    int inner = InnerMsg;
    int outer = OuterMsg;
    int front = FrontMsg;
    int lengh = InnerMsg;
    int x = center.x + lengh * Angle_cos[front];
    int y = center.y - lengh * Angle_sin[front];
    //avoid code dump
    //if (CenterXMsg<0|| CenterXMsg>600){
    //	CenterXMsg = 0; CenterYMsg = 0; InnerMsg = 0; OuterMsg = 0; FrontMsg = 0;
    //}
    //中心點
    circle(oframe, center, 1, Scalar(0, 255, 0), 1);
    //內圈
    circle(oframe, center, inner, Scalar(0, 0, 255), 1);
    //外圈
    circle(oframe, center, outer, Scalar(0, 255, 0), 1);
    //車頭角
    line(oframe, center, Point(x, y), Scalar(255, 0, 255), 1);
    return oframe;
}
cv::Mat Vision::ScanModel(const cv::Mat iframe)
{
    cv::Mat oframe = iframe.clone();

    int x, y;
    for (int radius = Magn_Near_StartMsg; radius <= Magn_Far_EndMsg; radius += Magn_Near_GapMsg)
    {
        for (int angle = 0; angle < 360; angle += Angle_Interval(radius))
        {
            //略過柱子
            if ((angle >= Unscaned_Angle[0] && angle <= Unscaned_Angle[1]) ||
                (angle >= Unscaned_Angle[2] && angle <= Unscaned_Angle[3]) ||
                (angle >= Unscaned_Angle[4] && angle <= Unscaned_Angle[5]) ||
                (angle >= Unscaned_Angle[6] && angle <= Unscaned_Angle[7]))
            {
                continue;
            }
            //掃描點的座標值
            x = Frame_Area(CenterXMsg + radius * Angle_cos[angle], oframe.cols);
            y = Frame_Area(CenterYMsg - radius * Angle_sin[angle], oframe.rows);
            //畫掃描點
            oframe.data[(y * oframe.cols + x) * 3 + 0] = 0;
            oframe.data[(y * oframe.cols + x) * 3 + 1] = 255;
            oframe.data[(y * oframe.cols + x) * 3 + 2] = 0;
        }
    }
    return oframe;
}
cv::Mat Vision::ColorModel(const cv::Mat iframe)
{

    cv::Mat oframe = iframe.clone();
    //cv::Mat hsv;
    //cvtColor(oframe,hsv,CV_BGR2HSV);

    int hmin, hmax, smin, smax, vmin, vmax;
    if (!HSV_red.empty() && !HSV_green.empty() && !HSV_blue.empty() && !HSV_yellow.empty() && !HSV_white.empty())
    {
        switch (ColorModeMsg)
        {
        case 0:

            hmin = HSV_red[0];
            hmax = HSV_red[1];
            smin = HSV_red[2];
            smax = HSV_red[3];
            vmin = HSV_red[4];
            vmax = HSV_red[5];
            break;
        case 1:
            hmin = HSV_green[0];
            hmax = HSV_green[1];
            smin = HSV_green[2];
            smax = HSV_green[3];
            vmin = HSV_green[4];
            vmax = HSV_green[5];
            break;
        case 2:
            hmin = HSV_blue[0];
            hmax = HSV_blue[1];
            smin = HSV_blue[2];
            smax = HSV_blue[3];
            vmin = HSV_blue[4];
            vmax = HSV_blue[5];
            break;
        case 3:
            hmin = HSV_yellow[0];
            hmax = HSV_yellow[1];
            smin = HSV_yellow[2];
            smax = HSV_yellow[3];
            vmin = HSV_yellow[4];
            vmax = HSV_yellow[5];
            break;
        case 4:
            hmin = HSV_white[0];
            hmax = HSV_white[1];
            smin = HSV_white[2];
            smax = HSV_white[3];
            vmin = HSV_white[4];
            vmax = HSV_white[5];
            break;
        }

        for (int i = 0; i < iframe.rows; i++)
        {
            for (int j = 0; j < iframe.cols; j++)
            {

                double B = iframe.data[(i * iframe.cols * 3) + (j * 3) + 0];
                double G = iframe.data[(i * iframe.cols * 3) + (j * 3) + 1];
                double R = iframe.data[(i * iframe.cols * 3) + (j * 3) + 2];
                double H, S, V;
                double Max = (max(R, G) > max(G, B)) ? max(R, G) : max(G, B); //max(R,G,B);
                double Min = (min(R, G) < min(G, B)) ? min(R, G) : min(G, B); //min(R,G,B);

                if (Max == Min)
                    Max += 1;
                if (R == Max)
                {
                    H = (G - B) * 60 / (Max - Min);
                }
                if (G == Max)
                {
                    H = 120 + (B - R) * 60 / (Max - Min);
                }
                if (B == Max)
                {
                    H = 240 + (R - G) * 60 / (Max - Min);
                }
                if (B == G && B == R)
                    H = 0;
                if (H < 0)
                {
                    H = H + 360;
                }
                S = (((Max - Min) * 100) / Max);
                if (Max == 0)
                    S = 0;
                V = Max / 2.56;

                switch (ColorModeMsg)
                {
                case 0:
                    if (hmax >= hmin)
                    {
                        if ((H <= hmax) && (H >= hmin) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin))
                        {
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 197;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 149;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 0;
                        }
                    }
                    else
                    {
                        if (((H <= hmax) || (H >= hmin)) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin))
                        {
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 197;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 149;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 0;
                        }
                    }
                    break;
                case 1:
                    if (hmax >= hmin)
                    {
                        if ((H <= hmax) && (H >= hmin) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin))
                        {
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 255;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 0;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 255;
                        }
                    }
                    else
                    {
                        if (((H <= hmax) || (H >= hmin)) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin))
                        {
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 255;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 2;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 255;
                        }
                    }
                    break;
                case 2:
                    if (hmax >= hmin)
                    {
                        if ((H <= hmax) && (H >= hmin) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin))
                        {
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 127;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 183;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 224;
                        }
                    }
                    else
                    {
                        if (((H <= hmax) || (H >= hmin)) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin))
                        {
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 127;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 183;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 224;
                        }
                    }
                    break;
                case 3:
                    if (hmax >= hmin)
                    {
                        if ((H <= hmax) && (H >= hmin) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin))
                        {
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 207;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 90;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 111;
                        }
                    }
                    else
                    {
                        if (((H <= hmax) || (H >= hmin)) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin))
                        {
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 207;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 90;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 111;
                        }
                    }
                    break;
                case 4:
                    if (hmax >= hmin)
                    {
                        if ((H <= hmax) && (H >= hmin) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin))
                        {
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 100;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 0;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 255;
                        }
                    }
                    else
                    {
                        if (((H <= hmax) || (H >= hmin)) && (S <= smax) && (S >= smin) && (V <= vmax) && (V >= vmin))
                        {
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 0] = 100;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 1] = 0;
                            oframe.data[(i * oframe.cols * 3) + (j * 3) + 2] = 255;
                        }
                    }
                    break;
                }
            }
        }
    }
    else
    {
        cout << "hsv data error" << endl;
    }
    return oframe;
}
cv::Mat Vision::White_Line(const cv::Mat iframe)
{
    cv::Mat threshold(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    cv::Mat oframe(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    //======================threshold===================
    for (int i = 0; i < iframe.rows; i++)
    {
        for (int j = 0; j < iframe.cols; j++)
        {
            int gray = (iframe.data[(i * iframe.cols * 3) + (j * 3) + 0] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 1] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 2]) / 3;
            if (gray <= WhiteGrayMsg)
            {
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 0] = 0;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 1] = 0;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 2] = 0;
            }
            else
            {
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 0] = 255;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 1] = 255;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 2] = 255;
            }
        }
    }
    //=====================draw the scan line===========
    oframe = threshold.clone();
    for (double angle = FrontMsg; angle < 360 + FrontMsg; angle += WhiteAngleMsg)
    {
        for (int r = InnerMsg; r <= OuterMsg; r++)
        {
            int angle_be = Angle_Adjustment(angle);

            int x_ = r * Angle_cos[angle_be];
            int y_ = r * Angle_sin[angle_be];
            int x = Frame_Area(CenterXMsg + x_, oframe.cols);
            int y = Frame_Area(CenterYMsg - y_, oframe.rows);

            if (threshold.data[(y * threshold.cols + x) * 3 + 0] != 255)
            {
                if (angle_be == FrontMsg)
                {
                    oframe.data[(y * oframe.cols + x) * 3 + 0] = 255;
                    oframe.data[(y * oframe.cols + x) * 3 + 1] = 255;
                    oframe.data[(y * oframe.cols + x) * 3 + 2] = 0;
                }
                else
                {
                    oframe.data[(y * oframe.cols + x) * 3 + 0] = 0;
                    oframe.data[(y * oframe.cols + x) * 3 + 1] = 0;
                    oframe.data[(y * oframe.cols + x) * 3 + 2] = 255;
                }
            }
            else
            {
                break;
            }
        }
    }
    line(oframe, Point(CenterXMsg, CenterYMsg - InnerMsg), Point(CenterXMsg, CenterYMsg + InnerMsg), Scalar(0, 255, 0), 1);
    line(oframe, Point(CenterXMsg - InnerMsg, CenterYMsg), Point(CenterXMsg + InnerMsg, CenterYMsg), Scalar(0, 255, 0), 1);
    circle(oframe, Point(CenterXMsg, CenterYMsg), InnerMsg, Scalar(0, 255, 0), 0);
    circle(oframe, Point(CenterXMsg, CenterYMsg), OuterMsg, Scalar(0, 255, 0), 0);

    return oframe;
}
cv::Mat Vision::Black_Line(const cv::Mat iframe)
{
    cv::Mat threshold(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    cv::Mat oframe(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    //======================threshold===================
    for (int i = 0; i < iframe.rows; i++)
    {
        for (int j = 0; j < iframe.cols; j++)
        {
            int gray = (iframe.data[(i * iframe.cols * 3) + (j * 3) + 0] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 1] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 2]) / 3;
            if (gray <= BlackGrayMsg)
            {
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 0] = 0;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 1] = 0;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 2] = 0;
            }
            else
            {
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 0] = 255;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 1] = 255;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 2] = 255;
            }
        }
    }
    //=====================draw the scan line===========
    oframe = threshold.clone();
    for (double angle = FrontMsg; angle < 360 + FrontMsg; angle = angle + BlackAngleMsg)
    {
        for (int r = InnerMsg; r <= OuterMsg; r++)
        {
            int angle_be = Angle_Adjustment(angle);

            int x_ = r * Angle_cos[angle_be];
            int y_ = r * Angle_sin[angle_be];
            int x = Frame_Area(CenterXMsg + x_, oframe.cols);
            int y = Frame_Area(CenterYMsg - y_, oframe.rows);

            if (threshold.data[(y * threshold.cols + x) * 3 + 0] != 0)
            {
                if (angle_be == FrontMsg)
                {
                    oframe.data[(y * oframe.cols + x) * 3 + 0] = 255;
                    oframe.data[(y * oframe.cols + x) * 3 + 1] = 255;
                    oframe.data[(y * oframe.cols + x) * 3 + 2] = 0;
                }
                else
                {
                    oframe.data[(y * oframe.cols + x) * 3 + 0] = 0;
                    oframe.data[(y * oframe.cols + x) * 3 + 1] = 0;
                    oframe.data[(y * oframe.cols + x) * 3 + 2] = 255;
                }
            }
            else
            {
                break;
            }
        }
    }
    line(oframe, Point(CenterXMsg, CenterYMsg - InnerMsg), Point(CenterXMsg, CenterYMsg + InnerMsg), Scalar(0, 255, 0), 1);
    line(oframe, Point(CenterXMsg - InnerMsg, CenterYMsg), Point(CenterXMsg + InnerMsg, CenterYMsg), Scalar(0, 255, 0), 1);
    circle(oframe, Point(CenterXMsg, CenterYMsg), InnerMsg, Scalar(0, 255, 0), 0);
    circle(oframe, Point(CenterXMsg, CenterYMsg), OuterMsg, Scalar(0, 255, 0), 0);

    return oframe;
}
