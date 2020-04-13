#include "obstacle.h"
#define TO_RAD M_PI/180.0
#define TO_DEG 180.0/M_PI
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
//=============================影像接收=======================================
void Vision::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //convert image data
        if (!cv_ptr->image.empty())
        {
            Source = cv_ptr->image.clone();
            cv::flip(Source, Source, 1); // reverse image
            monitor = Source.clone();
            threshold = Mat(Size(Source.cols, Source.rows), CV_8UC3, Scalar(0, 0, 0));

            FrameRate = Rate();

            Mat blackline = Black_Item(Source);
            //Mat test = ColorMoldel(Source, HSV_robot);
            //cv::imshow("Image window", cv_ptr->image);
            //cv::imshow("blackline", blackline);
            //cv::imshow("monitor", monitor);
            //cv::waitKey(10);
            Pub_obstacleframe(monitor);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert to image!");
        return;
    }
}
Mat Vision::ColorMoldel(Mat iframe, vector<int> HSV)
{
    Mat hsv(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    Mat mask(iframe.rows, iframe.cols, CV_8UC1, Scalar(0, 0, 0));
    Mat mask2(iframe.rows, iframe.cols, CV_8UC1, Scalar(0, 0, 0));
    Mat oframe(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));

    //cout<<HSV.size()<<endl;

    int hmin, hmax, smin, smax, vmin, vmax;
    if (HSV.size() == 6)
    {
        hmin = HSV[0]*0.5;
        hmax = HSV[1]*0.5;
        smin = HSV[2]*2.55;
        smax = HSV[3]*2.55;
        vmin = HSV[4]*2.55;
        vmax = HSV[5]*2.55;
        
        cvtColor(iframe, hsv, CV_BGR2HSV);
        if (HSV[0] <= HSV[1])
        {
            inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), mask);
        }
        else
        {
            inRange(hsv, Scalar(hmin, smin, vmin), Scalar(200, smax, vmax), mask);
            inRange(hsv, Scalar(10, smin, vmin), Scalar(hmax, smax, vmax), mask2);
            mask = mask + mask2;
        }

        //開操作 (去除一些噪點)
        Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
        morphologyEx(mask, mask, MORPH_OPEN, element);

        //閉操作 (連接一些連通域)
        element = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(mask, mask, MORPH_CLOSE, element);
        
        //iframe.copyTo(oframe, (cv::Mat::ones(mask.size(), mask.type()) * 255 - mask));
        oframe = convertTo3Channels(cv::Mat::ones(mask.size(), mask.type()) * 255 - mask);
        //cv::imshow("oframe", oframe);
        //waitKey(10);
    }
    else
    {
        cout << "HSV vector size: " << HSV.size() << " error\n";
    }
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
cv::Mat Vision::Black_Item(const cv::Mat iframe)
{
    //cv::Mat threshold(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    cv::Mat oframe(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    threshold = ColorMoldel(iframe, HSV_robot);
    //======================threshold===================

    /*for (int i = 0; i < iframe.rows; i++)
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
    }*/
    //=====================draw the scan line===========
    oframe = threshold.clone();
    
    Mat frame_ = Mat(Size(Source.cols, Source.rows), CV_8UC3, Scalar(0, 0, 0));
    DetectedObject FIND_Item;//找障礙物位置
    vector<DetectedObject> obj_item;
    deque<int> find_point;
    int color = 0;
    //cout<<BlackGrayMsg<<endl;
    
    int x, y;
    int x_, y_;
    int object_size;
    int dis, ang;
    int search_end = HorizonMsg;
    //int search_end = Magn_Far_StartMsg;
    for (int distance = InnerMsg; distance <= search_end; distance += Magn_Near_GapMsg)
    {
        for (int angle = 0; angle < 360; angle += Angle_Interval(distance))
        {
            int find_angle = Angle_Adjustment(angle);
            if ((find_angle >= Unscaned_Angle[0] && angle <= Unscaned_Angle[1]) ||
                (find_angle >= Unscaned_Angle[2] && angle <= Unscaned_Angle[3]) ||
                (find_angle >= Unscaned_Angle[4] && angle <= Unscaned_Angle[5]) ||
                (find_angle >= Unscaned_Angle[6] && angle <= Unscaned_Angle[7]))
            {
                continue;
            }

            object_size = 0;
            FIND_Item.size = 0;

            x_ = distance * Angle_cos[find_angle];
            y_ = distance * Angle_sin[find_angle];

            x = Frame_Area(CenterXMsg + x_, frame_.cols);
            y = Frame_Area(CenterYMsg - y_, frame_.rows);

            
            if (threshold.data[(y * threshold.cols + x) * 3 + 0] == 0&&
                threshold.data[(y * threshold.cols + x) * 3 + 1] == 0&&
                threshold.data[(y * threshold.cols + x) * 3 + 2] == 0 )
            {
                Mark_point(frame_, find_point, distance, find_angle, x, y, object_size, color);
                Mark_point(monitor, find_point, distance, find_angle, x, y, object_size, color);

                FIND_Item.dis_max = distance;
                FIND_Item.dis_min = distance;
                FIND_Item.ang_max = find_angle;
                FIND_Item.ang_min = find_angle;
                while (!find_point.empty())
                {
                    dis = find_point.front();
                    find_point.pop_front();

                    ang = find_point.front();
                    find_point.pop_front();
                    
                    object_compare(FIND_Item, dis, ang);
                    find_around_black(frame_, find_point, dis, ang, object_size, color);

                }
                FIND_Item.size = object_size;
            }

            find_point.clear();

            double ang_tmp = abs(FIND_Item.ang_max-FIND_Item.ang_min)/2;
            double obj_width = 2*Omni_distance(FIND_Item.dis_min)*sin(ang_tmp*TO_RAD)/cos(ang_tmp*TO_RAD);
            if(FIND_Item.size>10 && obj_width<80 && FIND_Item)
            //if (!(FIND_Item.size < 100 && FIND_Item.distance < 50))
            {
                
                obj_item.push_back(FIND_Item);
            }
        }
    }
    for(int i = 0; i < obj_item.size(); i++){
        int x, y;
        int x_, y_;

        int angle_, distance_;
        int find_angle;
        angle_ = Angle_Adjustment((obj_item.at(i).ang_max + obj_item.at(i).ang_min) / 2);
        distance_ = obj_item.at(i).dis_min;

        find_angle = Angle_Adjustment(angle_);

        x_ = distance_ * Angle_cos[find_angle];
        y_ = distance_ * Angle_sin[find_angle];

        x = Frame_Area(CenterXMsg + x_, Source.cols);
        y = Frame_Area(CenterYMsg - y_, Source.rows);    

        obj_item.at(i).x = x;
        obj_item.at(i).y = y;
        obj_item.at(i).distance = distance_;
        obj_item.at(i).angle = find_angle;
        
        draw_ellipse(monitor, obj_item.at(i), 5);
        circle(monitor, Point(x, y), 3, Scalar(0, 0, 200), -1);
    }
    pub_obstacle(obj_item);
    pub_pixel_obstacle(obj_item);
    //cv::imshow("threshold", threshold);
    //cv::imshow("frame_", frame_);
    //cv::waitKey(10);
    return oframe;
}
void Vision::Mark_point(Mat &frame_, deque<int> &find_point, int distance, int angle, int x, int y, int &size, int color)
{
    frame_.data[(y * frame_.cols + x) * 3 + 0] = 255;
    frame_.data[(y * frame_.cols + x) * 3 + 1] = 255;
    frame_.data[(y * frame_.cols + x) * 3 + 2] = 255;
    find_point.push_back(distance);
    find_point.push_back(angle);
    size += 1;
}
//判斷物件資料並且更新
void Vision::object_compare(DetectedObject &FIND_Item, int distance, int angle)
{
    if (FIND_Item.dis_max < distance)
    {
        FIND_Item.dis_max = distance;
    }
    if (FIND_Item.dis_min > distance)
    {
        FIND_Item.dis_min = distance;
    }

    if (FIND_Item.ang_max < angle)
    {
        FIND_Item.ang_max = angle;
    }
    if (FIND_Item.ang_min > angle)
    {
        FIND_Item.ang_min = angle;
    }
}
void Vision::find_around_black(Mat &frame_, deque<int> &find_point, int distance, int angle, int &size, int color)
{
    int x, y;
    int x_, y_;
    int dis_f, ang_f;
    double angle_f;
    int search_end = HorizonMsg;
    //int search_end = Magn_Far_StartMsg;
    for (int i = -1; i < 2; i++)
    {
        for (int j = -1; j < 2; j++)
        {
            dis_f = distance + i * Magn_Near_GapMsg;
            if (dis_f < InnerMsg)
                dis_f = InnerMsg;

            //dis_f = Frame_Area(dis_f, Magn_Far_EndMsg);
            if(dis_f>search_end)break;
            
            ang_f = angle + j * Angle_Interval(dis_f);
            ang_f = ang_f - (ang_f % Angle_Interval(dis_f));
            
            while ((Angle_Adjustment(ang_f) > Unscaned_Angle[0] && Angle_Adjustment(ang_f) < Unscaned_Angle[1]) ||
                   (Angle_Adjustment(ang_f) > Unscaned_Angle[2] && Angle_Adjustment(ang_f) < Unscaned_Angle[3]) ||
                   (Angle_Adjustment(ang_f) > Unscaned_Angle[4] && Angle_Adjustment(ang_f) < Unscaned_Angle[5]) ||
                   (Angle_Adjustment(ang_f) > Unscaned_Angle[6] && Angle_Adjustment(ang_f) < Unscaned_Angle[7]))
            {
                if (j < 0){
                    ang_f += -1 * Angle_Interval(dis_f);
                    ang_f = ang_f - (ang_f % Angle_Interval(dis_f));
                }
                else{
                    ang_f += 1 * Angle_Interval(dis_f);
                    ang_f = ang_f + (Angle_Interval(dis_f) - (ang_f % Angle_Interval(dis_f)) );
                }
            }
            
            
            angle_f = Angle_Adjustment(ang_f);

            x_ = dis_f * Angle_cos[angle_f];
            y_ = dis_f * Angle_sin[angle_f];

            x = Frame_Area(CenterXMsg + x_, frame_.cols);
            y = Frame_Area(CenterYMsg - y_, frame_.rows);
					
            if (threshold.data[(y * threshold.cols + x) * 3 + 0] == 0 && frame_.data[(y * frame_.cols + x) * 3 + 0] == 0)
            {
                Mark_point(frame_, find_point, dis_f, ang_f, x, y, size, color);
                Mark_point(monitor, find_point, dis_f, ang_f, x, y, size, color);	
            }
        }
    }
}
void Vision::draw_ellipse(Mat &frame_, DetectedObject &obj_, int color)
{
    ellipse(frame_, Point(CenterXMsg, CenterYMsg), Size(obj_.dis_min, obj_.dis_min), 0, 360 - obj_.ang_max, 360 - obj_.ang_min, Scalar(0, 255, 255), 1);
    ellipse(frame_, Point(CenterXMsg, CenterYMsg), Size(obj_.dis_max, obj_.dis_max), 0, 360 - obj_.ang_max, 360 - obj_.ang_min, Scalar(0, 255, 255), 1);
    draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_max, 5);
    draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_min, 5);

    circle(frame_, Point(obj_.x, obj_.y), 2, Scalar(0, 0, 0), -1);
}
void Vision::draw_Line(Mat &frame_, int obj_distance_max, int obj_distance_min, int obj_angle, int color)
{
    int x_, y_;
    double angle_f;
    int x[2], y[2];

    angle_f = Angle_Adjustment(obj_angle);

    x_ = obj_distance_min * Angle_cos[angle_f];
    y_ = obj_distance_min * Angle_sin[angle_f];

    x[0] = Frame_Area(CenterXMsg + x_, frame_.cols);
    y[0] = Frame_Area(CenterYMsg - y_, frame_.rows);

    x_ = obj_distance_max * Angle_cos[angle_f];
    y_ = obj_distance_max * Angle_sin[angle_f];

    x[1] = Frame_Area(CenterXMsg + x_, frame_.cols);
    y[1] = Frame_Area(CenterYMsg - y_, frame_.rows);

    line(frame_, Point(x[0], y[0]), Point(x[1], y[1]), Scalar(255, 255, 0), 1);
    if(color==5){
        line(frame_, Point(x[0], y[0]), Point(x[1], y[1]), Scalar(0, 255, 255), 1);
    }
}

