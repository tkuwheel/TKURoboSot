#include "black_item.h"
#include "omp.h"

Vision::Vision()
{
    //image_sub = nh.subscribe(VISION_TOPIC, 1,static_cast<void (Vision::*)(const sensor_msgs::ImageConstPtr&)>(&Vision::imageCb),this);
    image_sub = nh.subscribe(VISION_TOPIC, 1, &Vision::imageCb, this);
}
Vision::Vision(string topic)
{
    image_sub = nh.subscribe(topic, 1, &Vision::imageCb, this);
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

            Mat blackline = Black_Item(Source);
            //Pub_blackframe(blackline);
            Pub_blackdis(blackdis);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert to image!");
        return;
    }
}
cv::Mat Vision::Black_Item(const cv::Mat iframe)
{
    cv::Mat threshold(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    cv::Mat oframe(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    blackdis.data.clear();
    //======================threshold===================
    #pragma omp parallel for schedule(static) collapse(2)
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
    #pragma omp parallel for schedule(static) collapse(2)
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
            if (threshold.data[(y * threshold.cols + x) * 3 + 0] == 0 || r == OuterMsg)
            {
                blackdis.data.push_back(Omni_distance(r));
                break;
            }
        }
    }
    line(oframe, Point(CenterXMsg, CenterYMsg - InnerMsg), Point(CenterXMsg, CenterYMsg + InnerMsg), Scalar(0, 255, 0), 1);
    line(oframe, Point(CenterXMsg - InnerMsg, CenterYMsg), Point(CenterXMsg + InnerMsg, CenterYMsg), Scalar(0, 255, 0), 1);
    circle(oframe, Point(CenterXMsg, CenterYMsg), InnerMsg, Scalar(0, 255, 0), 0);
    circle(oframe, Point(CenterXMsg, CenterYMsg), OuterMsg, Scalar(0, 255, 0), 0);

    //cv::imshow("Black Item", oframe);
    //cv::waitKey(10);

    return oframe;
}
