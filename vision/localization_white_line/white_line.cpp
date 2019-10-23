#include "white_line.h"
#include "omp.h"

#define DEG2RAD  M_PI/180
Vision::Vision()
{
    //image_sub = nh.subscribe(VISION_TOPIC, 1,static_cast<void (Vision::*)(const sensor_msgs::ImageConstPtr&)>(&Vision::imageCb),this);
    image_sub = nh.subscribe(VISION_TOPIC, 1, &Vision::imageCb, this);
}
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
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //convert image data
        Source = cv_ptr->image.clone();
        cv::flip(Source, Source, 1); // reverse image
        //resize(Source,Source,Size(Source.cols/2,Source.rows/2),0,0,INTER_LINEAR);
        if (!cv_ptr->image.empty())
        {
            //setMouseCallback("Image window", onMouse, NULL);
            //cv::imshow("Image window", cv_ptr->image);
            //cv::waitKey(10);

            FrameRate = Rate();

            Mat whiteline = White_Line(Source);
            Pub_whiteframe(whiteline);
            Pub_whitedis(whitedis);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert to image!");
        return;
    }
}
cv::Mat Vision::White_Line(const cv::Mat iframe)
{
    cv::Mat threshold(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    cv::Mat edge;
    cv::Mat oframe(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    Mat visual_map(550, 750, CV_8UC3, Scalar(0,0,0));
    int ground = OuterMsg-100;
    if(HorizonMsg>0&&HorizonMsg<OuterMsg){
        ground = HorizonMsg;
    }
    vector<int> white_dis;
    whitedis.data.clear();
    //======================threshold===================
    #pragma omp parallel for schedule(static) collapse(2)
    for (int i = 0; i < iframe.rows; i++)
    {
        for (int j = 0; j < iframe.cols; j++)
        {
            unsigned char gray = (iframe.data[(i * iframe.cols * 3) + (j * 3) + 0] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 1] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 2]) / 3;
            if (gray < WhiteGrayMsg)
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
    //cv::imshow("threshold", threshold);
    //Canny(threshold, edge, 50, 150, 3);
    //edge=convertTo3Channels(edge);
    //cv::imshow("edge", edge);

//=========================
    int hmin, hmax, smin, smax, vmin, vmax;
    Mat hsv(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    Mat mask(iframe.rows, iframe.cols, CV_8UC1, Scalar(0, 0, 0));
    Mat mask2(iframe.rows, iframe.cols, CV_8UC1, Scalar(0, 0, 0));
    Mat dst(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    if (HSV_green.size() == 6)
    {
        hmin = HSV_green[0]/2;
        hmax = HSV_green[1]/2;
        smin = HSV_green[2]*2.55;
        smax = HSV_green[3]*2.55;
        vmin = HSV_green[4]*2.55;
        vmax = HSV_green[5]*2.55;
        /*for(int i =0; i<HSV_green.size(); i++)
        {
            cout<<HSV_green[i]<<" ";
        }
        cout<<endl;*/
        cvtColor(iframe, hsv, CV_BGR2HSV);

        if (HSV_green[0] <= HSV_green[1])
        {
            inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), mask);
        }
        else
        {
            inRange(hsv, Scalar(hmin, smin, vmin), Scalar(255, smax, vmax), mask);
            inRange(hsv, Scalar(0, smin, vmin), Scalar(hmax, smax, vmax), mask2);
            mask = mask + mask2;
        }

        //開操作 (去除一些噪點)
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(mask, mask, MORPH_OPEN, element);

        //閉操作 (連接一些連通域)
        morphologyEx(mask, mask, MORPH_CLOSE, element);
        //cv::imshow("mask", mask);
        iframe.copyTo(dst, (cv::Mat::ones(mask.size(), mask.type()) * 255 - mask));
        //cv::imshow("dst", dst);
        //waitKey(10);
    }
    else
    {
        cout << "HSV vector size: " << HSV_green.size() << " error\n";
    }
    int green_range[360];
    for(int i=0; i<360; i++){
        green_range[i]=0;
    }
    //mask=convertTo3Channels(mask);
    #pragma omp parallel for schedule(static) collapse(2)
    for (double angle = FrontMsg; angle < 360 + FrontMsg; angle = angle + WhiteAngleMsg)
    {
        for (int r = ground; r >= InnerMsg; r--)
        {
            int angle_be = Angle_Adjustment(angle);

            int x_ = r * Angle_cos[angle_be];
            int y_ = r * Angle_sin[angle_be];
            int x = Frame_Area(CenterXMsg + x_, oframe.cols);
            int y = Frame_Area(CenterYMsg - y_, oframe.rows);
            //if (mask.data[(y * mask.cols + x)*3 + 0] == 255)
            if (mask.data[(y * mask.cols + x)] == 255)
            {
                green_range[angle_be] = r-3;
                break;
            }
        }
    }
    //畫場地
    #pragma omp parallel for schedule(static) collapse(2)
    for (int i = 0; i < threshold.rows; i++)
    {
        for (int j = 0; j < threshold.cols; j++)
        {
            if (mask.data[(i * mask.cols + j)] == 255)
            {
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 0] = 0;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 1] = 200;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 2] = 0;
            }
        }
    }
    //=====================draw the scan line===========
    Mat input = threshold.clone();
    oframe = threshold.clone();
    //oframe = threshold.clone();
    int line_count = 0;
    #pragma omp parallel for schedule(static) collapse(2)
    for (double angle = FrontMsg; angle < 360 + FrontMsg; angle = angle + WhiteAngleMsg)
    {
        int count = 0;
        for (int r = InnerMsg; r <= ground; r++)
        {

            if(line_count%2==0&&r<(InnerMsg+ground)/2){
                r=(InnerMsg+ground)/2;
            }
            int angle_be = Angle_Adjustment(angle);

            int x_ = r * Angle_cos[angle_be];
            int y_ = r * Angle_sin[angle_be];
            int x = Frame_Area(CenterXMsg + x_, oframe.cols);
            int y = Frame_Area(CenterYMsg - y_, oframe.rows);
            if(green_range[angle_be] <= r){
                //cout<<"fuck"<<endl;
                break;
            }
            if (input.data[(y * input.cols + x) * 3 + 0] != 255)
            {
                if (angle_be == FrontMsg)
                {
                    oframe.data[(y * oframe.cols + x) * 3 + 0] = 255;
                    oframe.data[(y * oframe.cols + x) * 3 + 1] = 150;
                    oframe.data[(y * oframe.cols + x) * 3 + 2] = 0;
                }
                else
                {
                    oframe.data[(y * oframe.cols + x) * 3 + 0] = 0;
                    oframe.data[(y * oframe.cols + x) * 3 + 1] = 0;
                    oframe.data[(y * oframe.cols + x) * 3 + 2] = 255;
                }
            }

            if (input.data[(y * input.cols + x) * 3 + 0] == 255)
            {
                circle(oframe, Point(x, y), 2, Scalar(255, 0, 0), 1);
                
                x=visual_map.cols/2+Omni_distance(r)*cos((angle-FrontMsg)*DEG2RAD);
                y=visual_map.rows/2-Omni_distance(r)*sin((angle-FrontMsg)*DEG2RAD);
                circle(visual_map, Point(x, y), 2, Scalar(0, 255, 255), -1);

                x=Omni_distance(r)*cos((angle-FrontMsg)*DEG2RAD);
                y=-Omni_distance(r)*sin((angle-FrontMsg)*DEG2RAD);
                whitedis.data.push_back(x);
                whitedis.data.push_back(y);
                
                count++;
                //if(count>10)break;
                
                //white_dis.push_back(Omni_distance(r));
                break;
            }
        }
        line_count++;
    }
    line(oframe, Point(CenterXMsg, CenterYMsg - InnerMsg), Point(CenterXMsg, CenterYMsg + InnerMsg), Scalar(0, 255, 0), 1);
    line(oframe, Point(CenterXMsg - InnerMsg, CenterYMsg), Point(CenterXMsg + InnerMsg, CenterYMsg), Scalar(0, 255, 0), 1);
    circle(oframe, Point(CenterXMsg, CenterYMsg), InnerMsg, Scalar(0, 255, 0), 0);
    circle(oframe, Point(CenterXMsg, CenterYMsg), ground, Scalar(0, 255, 0), 0);
    circle(oframe, Point(CenterXMsg, CenterYMsg), OuterMsg, Scalar(0, 255, 0), 0);

    //draw the robot
    circle(visual_map, Point(visual_map.cols/2, visual_map.rows/2), 5, Scalar(0, 0, 255), 1);
    line(visual_map, Point(visual_map.cols/2,visual_map.rows/2), Point(visual_map.cols/2+5,visual_map.rows/2), Scalar(0,0,255), 1);
    //cv::imshow("visual_map", visual_map);
    //cv::imshow("white_line", oframe);
    //cv::waitKey(10);

    return oframe;
}

Mat Vision::convertTo3Channels(const Mat &binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows, binImg.cols, CV_8UC3);
    vector<Mat> channels;
    for (int i = 0; i < 3; i++)
    {
        channels.push_back(binImg);
    }
    merge(channels, three_channel);
    return three_channel;
}

