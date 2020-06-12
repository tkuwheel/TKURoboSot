#include "monitor.h"
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
//=============================影像接收=======================================
void Vision::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    //void Vision::imageCb(const sensor_msgs::CompressedImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {

        //static ros::Time start_time = ros::Time::now();
        //double sec = (ros::Time::now() - start_time).toSec();
        //if(sec>5){
        //    cout<<"in\n";
        //    while(1){
        //    //cout<<sec<<endl;
        //    }
        //}

        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //convert image data
        Source = cv_ptr->image.clone();

        cv::flip(Source, Source, 1); // reverse image
        //resize(Source,Source,Size(Source.cols/2,Source.rows/2),0,0,INTER_LINEAR);
        //if(!cv_ptr->image.empty()){
        //	cv::imshow("view", cv_ptr->image);
        //	cv::waitKey(10);
        //}
        FrameRate = Rate();
        RateMsg = FrameRate;
        Monitor = Source.clone();
        source2threshold();
        ObjectProcessing();
        Pub_monitor(Monitor);
        Pub_object();
        Pub_goal_edge();
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert to image!");
        return;
    }
}
void Vision::source2threshold(){
    //======================threshold===================
    Mat iframe = Source.clone();
    Mat threshold(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    for (int i = 0; i < iframe.rows; i++)
    {
        for (int j = 0; j < iframe.cols; j++)
        {
            unsigned char gray = (iframe.data[(i * iframe.cols * 3) + (j * 3) + 0] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 1] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 2]) / 3;
            if (gray < BlackGrayMsg)
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

    //開操作 (去除一些噪點)
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(threshold, threshold, MORPH_OPEN, element);

    //閉操作 (連接一些連通域)
    morphologyEx(threshold, threshold, MORPH_CLOSE, element);

    Threshold=threshold.clone();
}
//===============================物件分割=======================================
void Vision::ObjectProcessing()
{
    Red_Item.Reset();
    Blue_Item.Reset();
    Yellow_Item.Reset();
    draw_center();
    #pragma omp parallel sections num_threads(3)
    {
        #pragma omp section
        {
            objectdet_change(REDITEM, Red_Item);
        }
        #pragma omp section
        {
           objectdet_change(BLUEITEM, Blue_Item);
        }
        #pragma omp section
        {
           objectdet_change(YELLOWITEM, Yellow_Item);
        }
    }
}
void Vision::objectdet_change( int color, DetectedObject &obj_item)
{
    int x, y;
    int x_, y_;
    int object_size;
    int dis, ang;

    Mat frame_ = Mat(Size(Source.cols, Source.rows), CV_8UC3, Scalar(0, 0, 0));

    DetectedObject FIND_Item;
    deque<int> find_point;
    //find_point.clear();
    FIND_Item.Reset();

    for (int distance = Magn_Near_StartMsg; distance <= Magn_Far_EndMsg; distance += Magn_Near_GapMsg)
    {
        for (int angle = 0; angle < 360; angle += Angle_Interval(distance))
        {
            if ((angle >= Unscaned_Angle[0] && angle <= Unscaned_Angle[1]) ||
                (angle >= Unscaned_Angle[2] && angle <= Unscaned_Angle[3]) ||
                (angle >= Unscaned_Angle[4] && angle <= Unscaned_Angle[5]) ||
                (angle >= Unscaned_Angle[6] && angle <= Unscaned_Angle[7]))
            {
                continue;
            }
            object_size = 0;
            FIND_Item.size = 0;

            x_ = distance * Angle_cos[angle];
            y_ = distance * Angle_sin[angle];

            x = Frame_Area(CenterXMsg + x_, frame_.cols);
            y = Frame_Area(CenterYMsg - y_, frame_.rows);

            unsigned char B = Source.data[(y * Source.cols + x) * 3 + 0];
            unsigned char G = Source.data[(y * Source.cols + x) * 3 + 1];
            unsigned char R = Source.data[(y * Source.cols + x) * 3 + 2];
            if (color_map[R + (G << 8) + (B << 16)] & color && frame_.data[(y * frame_.cols + x) * 3 + 0] == 0)
            {
                Mark_point(frame_, find_point, distance, angle, x, y, object_size, color);

                FIND_Item.dis_max = distance;
                FIND_Item.dis_min = distance;
                FIND_Item.ang_max = angle;
                FIND_Item.ang_min = angle;
                while (!find_point.empty())
                {
                    dis = find_point.front();
                    find_point.pop_front();

                    ang = find_point.front();
                    find_point.pop_front();

                    object_compare(FIND_Item, dis, ang);
                    find_around(frame_, find_point, dis, ang, object_size, color);

                }
                FIND_Item.size = object_size;
            }

            find_point.clear();

            if (FIND_Item.size > obj_item.size)
            {
                obj_item = FIND_Item;
            }
        }
    }

    // if (obj_item.size > SizeFilter)
    if (obj_item.size > 0)
    {
        find_object_point(obj_item, color);
        draw_ellipse(Monitor, obj_item, color);
        if (color == BLUEITEM || color == YELLOWITEM)
        {
            find_edge_point(obj_item, color);
            find_shoot_point(obj_item, color);
        }
        if (color == REDITEM)
            draw_point(Monitor, Red_Item, "R", Scalar(0, 0, 255));
        if (color == BLUEITEM)
            draw_point(Monitor, Blue_Item, "B", Scalar(255, 0, 0));
        if (color == YELLOWITEM)
            draw_point(Monitor, Yellow_Item, "Y", Scalar(0, 255, 255));
    }
    //imshow("findmap", frame_);
    //waitKey(10);
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
//尋找當前位置周圍座標點
void Vision::find_around(Mat &frame_, deque<int> &find_point, int distance, int angle, int &size, int color)
{
    int x, y;
    int x_, y_;
    int dis_f, ang_f;
    double angle_f;

    for (int i = -1; i < 2; i++)
    {
        for (int j = -1; j < 2; j++)
        {
            dis_f = distance + i*Magn_Near_GapMsg;

            if (dis_f < Magn_Near_StartMsg)
                dis_f = Magn_Near_StartMsg;

            dis_f = Frame_Area(dis_f, Magn_Far_EndMsg);
            ang_f = angle + j * Angle_Interval(dis_f);

            while ((Angle_Adjustment(ang_f) > Unscaned_Angle[0] && Angle_Adjustment(ang_f) < Unscaned_Angle[1]) ||
                   (Angle_Adjustment(ang_f) > Unscaned_Angle[2] && Angle_Adjustment(ang_f) < Unscaned_Angle[3]) ||
                   (Angle_Adjustment(ang_f) > Unscaned_Angle[4] && Angle_Adjustment(ang_f) < Unscaned_Angle[5]) ||
                   (Angle_Adjustment(ang_f) > Unscaned_Angle[6] && Angle_Adjustment(ang_f) < Unscaned_Angle[7]))
            {
                if (j < 0)
                    ang_f += -1 * Angle_Interval(dis_f);
                else
                    ang_f += 1 * Angle_Interval(dis_f);
            }

            angle_f = Angle_Adjustment(ang_f);

            x_ = dis_f * Angle_cos[angle_f];
            y_ = dis_f * Angle_sin[angle_f];

            x = Frame_Area(CenterXMsg + x_, frame_.cols);
            y = Frame_Area(CenterYMsg - y_, frame_.rows);

            unsigned char B = Source.data[(y * Source.cols + x) * 3 + 0];
            unsigned char G = Source.data[(y * Source.cols + x) * 3 + 1];
            unsigned char R = Source.data[(y * Source.cols + x) * 3 + 2];

            if (color_map[R + (G << 8) + (B << 16)] & color && frame_.data[(y * frame_.cols + x) * 3 + 0] == 0)
            {
                Mark_point(frame_, find_point, dis_f, ang_f, x, y, size, color);
            }
        }
    }
}
void Vision::find_around_black(Mat &frame_, deque<int> &find_point, int distance, int angle, int &size, int color)
{
    int x, y;
    int x_, y_;
    int dis_f, ang_f;
    double angle_f;

    for (int i = -1; i < 2; i++)
    {
        for (int j = -1; j < 2; j++)
        {
            dis_f = distance + i * Magn_Near_GapMsg;

            if (dis_f < Magn_Near_StartMsg)
                dis_f = Magn_Near_StartMsg;

            dis_f = Frame_Area(dis_f, Magn_Far_EndMsg);
            ang_f = angle + j * Angle_Interval(dis_f);

            while ((Angle_Adjustment(ang_f) > Unscaned_Angle[0] && Angle_Adjustment(ang_f) < Unscaned_Angle[1]) ||
                   (Angle_Adjustment(ang_f) > Unscaned_Angle[2] && Angle_Adjustment(ang_f) < Unscaned_Angle[3]) ||
                   (Angle_Adjustment(ang_f) > Unscaned_Angle[4] && Angle_Adjustment(ang_f) < Unscaned_Angle[5]) ||
                   (Angle_Adjustment(ang_f) > Unscaned_Angle[6] && Angle_Adjustment(ang_f) < Unscaned_Angle[7]))
            {
                if (j < 0)
                    ang_f += -1 * Angle_Interval(dis_f);
                else
                    ang_f += 1 * Angle_Interval(dis_f);
            }

            angle_f = Angle_Adjustment(ang_f);

            x_ = dis_f * Angle_cos[angle_f];
            y_ = dis_f * Angle_sin[angle_f];

            x = Frame_Area(CenterXMsg + x_, frame_.cols);
            y = Frame_Area(CenterYMsg - y_, frame_.rows);
					
            if (Threshold.data[(y * Threshold.cols + x) * 3 + 0] == 0 && frame_.data[(y * frame_.cols + x) * 3 + 0] == 0)
            {
                Mark_point(frame_, find_point, dis_f, ang_f, x, y, size, color);	
            }
        }
    }
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
//找物體中心點
void Vision::find_object_point(DetectedObject &obj_, int color)
{
    int x, y;
    int x_, y_;

    int angle_, distance_;
    int angle_range;
    int find_angle;

    unsigned char B, G, R;

    if (color == REDITEM)
    {
        //cout<<obj_.dis_max<<"  "<<obj_.dis_max-obj_.dis_min<<endl;
        //fix catch ball distance        
        if(obj_.dis_max<70){
            obj_.dis_min = obj_.dis_max - 26;
        }
        angle_ = Angle_Adjustment((obj_.ang_max + obj_.ang_min) / 2);
        distance_ = obj_.dis_min;

        find_angle = Angle_Adjustment(angle_);

        x_ = distance_ * Angle_cos[find_angle];
        y_ = distance_ * Angle_sin[find_angle];

        x = Frame_Area(CenterXMsg + x_, Source.cols);
        y = Frame_Area(CenterYMsg - y_, Source.rows);    

        obj_.x = x;
        obj_.y = y;
        obj_.distance = distance_;
        obj_.angle = find_angle;
    }
    else if (color == BLUEITEM || color == YELLOWITEM)
    {
        angle_ = Angle_Adjustment((obj_.ang_max + obj_.ang_min) / 2);
        distance_ = obj_.dis_min;

        find_angle = Angle_Adjustment(angle_);

        x_ = distance_ * Angle_cos[find_angle];
        y_ = distance_ * Angle_sin[find_angle];

        x = Frame_Area(CenterXMsg + x_, Source.cols);
        y = Frame_Area(CenterYMsg - y_, Source.rows);    

        obj_.x = x;
        obj_.y = y;
        obj_.distance = distance_;
        obj_.angle = find_angle;
        /*
        angle_ = Angle_Adjustment((obj_.ang_max + obj_.ang_min) / 2);
        angle_range = 0.7 * Angle_Adjustment((obj_.ang_max - obj_.ang_min) / 2);

        for (int distance = obj_.dis_min; distance <= obj_.dis_max; distance++)
        {
            for (int angle = 0; angle <= angle_range; angle++)
            {
                find_angle = Angle_Adjustment(angle_ + angle);

                x_ = distance * Angle_cos[find_angle];
                y_ = distance * Angle_sin[find_angle];

                x = Frame_Area(CenterXMsg + x_, Source.cols);
                y = Frame_Area(CenterYMsg - y_, Source.rows);

                B = Source.data[(y * Source.cols + x) * 3 + 0];
                G = Source.data[(y * Source.cols + x) * 3 + 1];
                R = Source.data[(y * Source.cols + x) * 3 + 2];

                if (color_map[R + (G << 8) + (B << 16)] & color)
                {
                    obj_.x = x;
                    obj_.y = y;
                    obj_.distance = distance;
                    obj_.angle = find_angle;
                    break;
                }

                find_angle = Angle_Adjustment(angle_ - angle);

                x_ = distance * Angle_cos[find_angle];
                y_ = distance * Angle_sin[find_angle];

                x = Frame_Area(CenterXMsg + x_, Source.cols);
                y = Frame_Area(CenterYMsg - y_, Source.rows);

                B = Source.data[(y * Source.cols + x) * 3 + 0];
                G = Source.data[(y * Source.cols + x) * 3 + 1];
                R = Source.data[(y * Source.cols + x) * 3 + 2];

                if (color_map[R + (G << 8) + (B << 16)] & color)
                {
                    obj_.x = x;
                    obj_.y = y;
                    obj_.distance = distance;
                    obj_.angle = find_angle;
                    break;
                }
            }
            if (obj_.distance != 0)
            {
                break;
            }
        }
        */
    }

    if (Angle_Adjustment(angle_ - FrontMsg) < 180)
    {
        obj_.LR = "Left";
    }
    else
    {
        obj_.LR = "Right";
    }
}
//找球門左右邊界點
void Vision::find_edge_point(DetectedObject &obj_, int color)
{
    int x, y;
    int x_, y_;
    int right_angle, left_angle;
    int temp;
    int find_angle;
    unsigned char B, G, R;
    right_angle = obj_.ang_max;
    left_angle = obj_.ang_min;

    //right
    temp = 999;
    for (int angle = 0; angle < 5; angle++)
    {
        for (int distance = obj_.dis_min; distance <= obj_.dis_max; distance++)
        {
            find_angle = Angle_Adjustment(right_angle - angle);
            x_ = distance * Angle_cos[find_angle];
            y_ = distance * Angle_sin[find_angle];

            x = Frame_Area(CenterXMsg + x_, Source.cols);
            y = Frame_Area(CenterYMsg - y_, Source.rows);

            B = Source.data[(y * Source.cols + x) * 3 + 0];
            G = Source.data[(y * Source.cols + x) * 3 + 1];
            R = Source.data[(y * Source.cols + x) * 3 + 2];

            if (color_map[R + (G << 8) + (B << 16)] & color)
            {
                temp = distance;
                if (temp < obj_.right_dis)
                {
                    obj_.right_x = x;
                    obj_.right_y = y;
                    obj_.right_dis = temp;
                }
            }
        }
    }
    //left
    temp = 999;
    for (int angle = 0; angle < 5; angle++)
    {
        for (int distance = obj_.dis_min; distance <= obj_.dis_max; distance++)
        {
            find_angle = Angle_Adjustment(left_angle + angle);
            x_ = distance * Angle_cos[find_angle];
            y_ = distance * Angle_sin[find_angle];

            x = Frame_Area(CenterXMsg + x_, Source.cols);
            y = Frame_Area(CenterYMsg - y_, Source.rows);

            B = Source.data[(y * Source.cols + x) * 3 + 0];
            G = Source.data[(y * Source.cols + x) * 3 + 1];
            R = Source.data[(y * Source.cols + x) * 3 + 2];

            if (color_map[R + (G << 8) + (B << 16)] & color)
            {
                temp = distance;
                if (temp < obj_.left_dis)
                {
                    obj_.left_x = x;
                    obj_.left_y = y;
                    obj_.left_dis = distance;
                }
            }
        }
    }
    if (obj_.right_dis == 999)
    {
        x_ = obj_.dis_min * Angle_cos[obj_.ang_max];
        y_ = obj_.dis_min * Angle_sin[obj_.ang_max];

        x = Frame_Area(CenterXMsg + x_, Source.cols);
        y = Frame_Area(CenterYMsg - y_, Source.rows);
        obj_.right_x = x;
        obj_.right_y = y;
        obj_.right_dis = obj_.dis_min;
    }
    if (obj_.left_dis == 999)
    {
        x_ = obj_.dis_min * Angle_cos[obj_.ang_min];
        y_ = obj_.dis_min * Angle_sin[obj_.ang_min];

        x = Frame_Area(CenterXMsg + x_, Source.cols);
        y = Frame_Area(CenterYMsg - y_, Source.rows);
        obj_.left_x = x;
        obj_.left_y = y;
        obj_.left_dis = obj_.dis_min;
    }
}
//找球門射擊點
void Vision::find_shoot_point(DetectedObject &obj_, int color)
{
    
    int x, y;
    int x_, y_;
    int object_size;
    int dis, ang;

    
    Mat iframe = Source.clone();
    Mat frame_(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    Mat threshold=Threshold.clone();
    DetectedObject FIND_Item;//找守門員位置
    DetectedObject obj_item;
    deque<int> find_point;
    //cout<<BlackGrayMsg<<endl;
    
    
    int angle_min = obj_.ang_min;
    int angle_max = obj_.ang_max;
    if(obj_.ang_min<0){
        angle_min = angle_min+360;
        angle_max = angle_max+360;
    }
    for (int distance = InnerMsg; distance <= obj_.dis_max; distance += Magn_Near_GapMsg)
    {
        for (int angle = angle_min-5; angle < angle_max+5; angle += Angle_Interval(distance))
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
            if(FIND_Item.ang_min<obj_.ang_min){
                FIND_Item.ang_min=FIND_Item.ang_min+360;
                FIND_Item.ang_max=FIND_Item.ang_max+360;
            }
            if(FIND_Item.ang_max>obj_.ang_max){
                FIND_Item.ang_min=FIND_Item.ang_min-360;
                FIND_Item.ang_max=FIND_Item.ang_max-360;
            }
            if (FIND_Item.size > obj_item.size&&FIND_Item.ang_min>=obj_.ang_min&&FIND_Item.ang_max<=obj_.ang_max)
            {
                obj_item = FIND_Item;
            }
        }
    }

    if(obj_item.ang_min<obj_.ang_min){
        obj_item.ang_min=obj_item.ang_min+360;
        obj_item.ang_max=obj_item.ang_max+360;
    }
    if(obj_item.ang_max>obj_.ang_max){
        obj_item.ang_min=obj_item.ang_min-360;
        obj_item.ang_max=obj_item.ang_max-360;
    }
    draw_ellipse2(Monitor, obj_item, 5);
    //if (color == BLUEITEM){
    //    cv::imshow("threshold", threshold);
    //    waitKey(10);
    //    cv::imshow("findmap", frame_);
    //    waitKey(10);
    //    cout<<obj_item.ang_min<<" "<<obj_item.ang_max<<" "<<obj_.ang_min<<" "<<obj_.ang_max<<endl;
    //}

    //========================================找最大範圍==============================
    //int x, y;
    //int x_, y_;
    int angle_;
    int angle_range;
    int find_angle;
    unsigned char B, G, R;
    int find_gap[2][7] = {0};
    int start = obj_.dis_min;
/*
    if (obj_.dis_min > 130)
    {
        start = obj_.dis_min - 20;
    }
    for (int angle = obj_.ang_min; angle <= obj_.ang_max; angle++)
    {
        for (int distance = start; distance <= (obj_.dis_min + obj_.dis_max) / 2; distance++)
        {
            find_angle = Angle_Adjustment(angle);

            if ((find_angle >= Unscaned_Angle[0] && find_angle <= Unscaned_Angle[1]) ||
                (find_angle >= Unscaned_Angle[2] && find_angle <= Unscaned_Angle[3]) ||
                (find_angle >= Unscaned_Angle[4] && find_angle <= Unscaned_Angle[5]) ||
                (find_angle >= Unscaned_Angle[6] && find_angle <= Unscaned_Angle[7]))
            {
                if (angle != obj_.ang_max)
                    break;
            }
            //中心座標
            x_ = distance * Angle_cos[find_angle];
            y_ = distance * Angle_sin[find_angle];
            //實際座標
            x = Frame_Area(CenterXMsg + x_, Source.cols);
            y = Frame_Area(CenterYMsg - y_, Source.rows);

            B = Source.data[(y * Source.cols + x) * 3 + 0];
            G = Source.data[(y * Source.cols + x) * 3 + 1];
            R = Source.data[(y * Source.cols + x) * 3 + 2];

            if (color_map[R + (G << 8) + (B << 16)] & color)
            {
                if (find_gap[1][0] == 0)
                {
                    find_gap[1][0] = x;
                    find_gap[1][1] = y;
                    find_gap[1][2] = angle;
                }
                else
                {
                    find_gap[1][3] = x;
                    find_gap[1][4] = y;
                    find_gap[1][5] = angle;
                }
                break;
            }

            if (color_map[R + (G << 8) + (B << 16)] & WHITEITEM || angle == obj_.ang_max)
            {
                Monitor.data[(y * Monitor.cols + x) * 3 + 0] = 255;
                Monitor.data[(y * Monitor.cols + x) * 3 + 1] = 255;
                Monitor.data[(y * Monitor.cols + x) * 3 + 2] = 255;
                find_gap[1][6] = find_gap[1][5] - find_gap[1][2];
                if (find_gap[0][6] < find_gap[1][6])
                {
                    if (color == BLUEITEM)
                    {
                        if (b_end_gap > 0 && b_end_gap < 720 &&
                            find_gap[1][6] < ((obj_.ang_max - obj_.ang_min) * 0.4) &&
                            (abs(find_gap[1][5] + find_gap[1][2]) / 2 - b_end_gap) > ((obj_.ang_max - obj_.ang_min) * 0.3))
                        {
                        }
                        else
                        {
                            for (int i = 0; i < 7; i++)
                            {
                                find_gap[0][i] = find_gap[1][i];
                            }
                        }
                    }
                    if (color == YELLOWITEM)
                    {
                        if (y_end_gap > 0 && y_end_gap < 720 &&
                            find_gap[1][6] < ((obj_.ang_max - obj_.ang_min) * 0.4) &&
                            (abs(find_gap[1][5] + find_gap[1][2]) / 2 - y_end_gap) > ((obj_.ang_max - obj_.ang_min) * 0.3))
                        {
                        }
                        else
                        {
                            for (int i = 0; i < 7; i++)
                            {
                                find_gap[0][i] = find_gap[1][i];
                            }
                        }
                    }
                }
                for (int i = 0; i < 7; i++)
                {
                    find_gap[1][i] = 0;
                }
                break;
            }
        }
    }
*/
    //===================================
    int right_gap = obj_.ang_max-obj_item.ang_max;
    int left_gap = obj_item.ang_min-obj_.ang_min;
    
    if(obj_item.ang_max!=0||obj_item.ang_min!=0){
      if(fabs(right_gap-left_gap)>fabs(obj_.ang_max-obj_.ang_min)*0.2){
        if(right_gap>left_gap){
            find_gap[0][2] = obj_item.ang_max;
            find_gap[0][5] = obj_.ang_max;
        }else{
            find_gap[0][2] = obj_.ang_min;
            find_gap[0][5] = obj_item.ang_min;
        }
        find_gap[0][6] = find_gap[0][5] - find_gap[0][2];
      }else{
        double right_center=Angle_Adjustment((obj_.ang_max+obj_item.ang_max)/2);
        double left_center=Angle_Adjustment((obj_item.ang_min+obj_.ang_min)/2);
        if(fabs(right_center-FrontMsg)<fabs(left_center-FrontMsg)){
            find_gap[0][2] = obj_item.ang_max;
            find_gap[0][5] = obj_.ang_max;
        }else{
            find_gap[0][2] = obj_.ang_min;
            find_gap[0][5] = obj_item.ang_min;          
        }
        find_gap[0][6] = find_gap[0][5] - find_gap[0][2];
      }
    }
    //==================================
    /*
    if (color == BLUEITEM)
    {
        if (b_end_gap > 0 && b_end_gap < 720 &&
            find_gap[1][6] < ((obj_.ang_max - obj_.ang_min) * 0.4) &&
            (abs(find_gap[1][5] + find_gap[1][2]) / 2 - b_end_gap) > ((obj_.ang_max - obj_.ang_min) * 0.3))
        {
        }
        else
        {
            for (int i = 0; i < 7; i++)
            {
                find_gap[0][i] = find_gap[1][i];
            }
        }
    }
    if (color == YELLOWITEM)
    {
        if (y_end_gap > 0 && y_end_gap < 720 &&
            find_gap[1][6] < ((obj_.ang_max - obj_.ang_min) * 0.4) &&
            (abs(find_gap[1][5] + find_gap[1][2]) / 2 - y_end_gap) > ((obj_.ang_max - obj_.ang_min) * 0.3))
        {
        }
        else
        {
            for (int i = 0; i < 7; i++)
            {
                find_gap[0][i] = find_gap[1][i];
            }
        }
    }
    */
    //==================================
    obj_.fix_ang_min = find_gap[0][2];
    obj_.fix_ang_max = find_gap[0][5];

    if (color == BLUEITEM)
    {
        if (find_gap[0][5] > 0)
            b_end_gap = (find_gap[0][5] + find_gap[0][2]) / 2;
        else
            b_end_gap = (obj_.ang_max + obj_.ang_min) / 2;
        //if(b_end_gap > obj_.ang_max || b_end_gap < obj_.ang_min)(obj_.ang_max + obj_.ang_min) / 2;
    }
    if (color == YELLOWITEM)
    {
        if (find_gap[0][5] > 0)
            y_end_gap = (find_gap[0][5] + find_gap[0][2]) / 2;
        else
            y_end_gap = (obj_.ang_max + obj_.ang_min) / 2;
        //if(y_end_gap > obj_.ang_max || y_end_gap < obj_.ang_min)(obj_.ang_max + obj_.ang_min) / 2;
    }
    //=============================找中心===================================
    
    angle_ = Angle_Adjustment((find_gap[0][5] + find_gap[0][2]) / 2);
    //angle_ = Angle_Adjustment((obj_item.ang_min + obj_item.ang_max) / 2);
    angle_range = 0.7 * Angle_Adjustment((find_gap[0][5] - find_gap[0][2]) / 2);
    for (int angle = 0; angle < angle_range; angle++)
    {
        for (int distance = obj_.dis_min; distance <= (obj_.dis_min + obj_.dis_max) / 2; distance++)
        {
            if (obj_.fix_distance != 0)
                break;
            find_angle = Angle_Adjustment(angle_ + angle);

            x_ = distance * Angle_cos[find_angle];
            y_ = distance * Angle_sin[find_angle];

            x = Frame_Area(CenterXMsg + x_, Source.cols);
            y = Frame_Area(CenterYMsg - y_, Source.rows);

            B = Source.data[(y * Source.cols + x) * 3 + 0];
            G = Source.data[(y * Source.cols + x) * 3 + 1];
            R = Source.data[(y * Source.cols + x) * 3 + 2];

            if (color_map[R + (G << 8) + (B << 16)] & color)
            {
                obj_.fix_x = x;
                obj_.fix_y = y;
                obj_.fix_distance = distance;
                obj_.fix_angle = angle_;
            }
            find_angle = Angle_Adjustment(angle_ - angle);

            x_ = distance * Angle_cos[find_angle];
            y_ = distance * Angle_sin[find_angle];

            x = Frame_Area(CenterXMsg + x_, Source.cols);
            y = Frame_Area(CenterYMsg - y_, Source.rows);

            B = Source.data[(y * Source.cols + x) * 3 + 0];
            G = Source.data[(y * Source.cols + x) * 3 + 1];
            R = Source.data[(y * Source.cols + x) * 3 + 2];

            if (color_map[R + (G << 8) + (B << 16)] & color)
            {
                obj_.fix_x = x;
                obj_.fix_y = y;
                obj_.fix_distance = distance;
                obj_.fix_angle = angle_;
            }
        }
    }

    if (obj_.fix_distance == 0)
    {
        obj_.fix_x = obj_.x;
        obj_.fix_y = obj_.y;
        obj_.fix_angle = obj_.angle;
        obj_.fix_distance = obj_.distance;
        obj_.fix_ang_min = obj_.ang_min;
        obj_.fix_ang_max = obj_.ang_max;
    }
}
void Vision::draw_ellipse(Mat &frame_, DetectedObject &obj_, int color)
{
    ellipse(frame_, Point(CenterXMsg, CenterYMsg), Size(obj_.dis_min, obj_.dis_min), 0, 360 - obj_.ang_max, 360 - obj_.ang_min, Scalar(255, 255, 0), 1);
    ellipse(frame_, Point(CenterXMsg, CenterYMsg), Size(obj_.dis_max, obj_.dis_max), 0, 360 - obj_.ang_max, 360 - obj_.ang_min, Scalar(255, 255, 0), 1);
    draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_max, color);
    draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_min, color);
    circle(frame_, Point(obj_.x, obj_.y), 2, Scalar(0, 0, 0), -1);
}
void Vision::draw_ellipse2(Mat &frame_, DetectedObject &obj_, int color)
{
    //goalkeeper
    ellipse(frame_, Point(CenterXMsg, CenterYMsg), Size(obj_.dis_min, obj_.dis_min), 0, 360 - obj_.ang_max, 360 - obj_.ang_min, Scalar(0, 255, 255), 1);
    ellipse(frame_, Point(CenterXMsg, CenterYMsg), Size(obj_.dis_max, obj_.dis_max), 0, 360 - obj_.ang_max, 360 - obj_.ang_min, Scalar(0, 255, 255), 1);
    draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_max, 5);
    draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_min, 5);
    circle(frame_, Point(obj_.x, obj_.y), 2, Scalar(0, 0, 0), -1);
}
void Vision::draw_center()
{
    int x, y;
    circle(Monitor, Point(CenterXMsg, CenterYMsg), 1, Scalar(0, 255, 0), 1);
    circle(Monitor, Point(CenterXMsg, CenterYMsg), InnerMsg, Scalar(0, 0, 255), 1);
    circle(Monitor, Point(CenterXMsg, CenterYMsg), OuterMsg, Scalar(0, 255, 0), 1);
    x = CenterXMsg + InnerMsg * cos(FrontMsg * PI / 180);
    y = CenterYMsg - InnerMsg * sin(FrontMsg * PI / 180);
    line(Monitor, Point(CenterXMsg, CenterYMsg), Point(x, y), Scalar(255, 0, 255), 1);

    x = CenterXMsg + OuterMsg * cos(Unscaned_Angle[0] * PI / 180);
    y = CenterYMsg - OuterMsg * sin(Unscaned_Angle[0] * PI / 180);
    line(Monitor, Point(CenterXMsg, CenterYMsg), Point(x, y), Scalar(255, 255, 255), 1);
    
    x = CenterXMsg + OuterMsg * cos(Unscaned_Angle[1] * PI / 180);
    y = CenterYMsg - OuterMsg * sin(Unscaned_Angle[1] * PI / 180);
    line(Monitor, Point(CenterXMsg, CenterYMsg), Point(x, y), Scalar(255, 255, 255), 1);
    
    x = CenterXMsg + OuterMsg * cos(Unscaned_Angle[2] * PI / 180);
    y = CenterYMsg - OuterMsg * sin(Unscaned_Angle[2] * PI / 180);
    line(Monitor, Point(CenterXMsg, CenterYMsg), Point(x, y), Scalar(255, 255, 255), 1);
    
    x = CenterXMsg + OuterMsg * cos(Unscaned_Angle[3] * PI / 180);
    y = CenterYMsg - OuterMsg * sin(Unscaned_Angle[3] * PI / 180);
    line(Monitor, Point(CenterXMsg, CenterYMsg), Point(x, y), Scalar(255, 255, 255), 1);
    
    x = CenterXMsg + OuterMsg * cos(Unscaned_Angle[4] * PI / 180);
    y = CenterYMsg - OuterMsg * sin(Unscaned_Angle[4] * PI / 180);
    line(Monitor, Point(CenterXMsg, CenterYMsg), Point(x, y), Scalar(255, 255, 255), 1);
    
    x = CenterXMsg + OuterMsg * cos(Unscaned_Angle[5] * PI / 180);
    y = CenterYMsg - OuterMsg * sin(Unscaned_Angle[5] * PI / 180);
    line(Monitor, Point(CenterXMsg, CenterYMsg), Point(x, y), Scalar(255, 255, 255), 1);

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
void Vision::draw_point(cv::Mat &frame_, DetectedObject &obj_, string color, Scalar Textcolor)
{
    std::string X;
    std::string Y;
    std::stringstream X_out;
    std::stringstream Y_out;

    //==========中心點繪製===============
    circle(frame_, Point(obj_.x, obj_.y), 2, Scalar(0, 0, 255), -1);
    X_out << obj_.x - CenterXMsg;
    Y_out << 0 - (obj_.y - CenterYMsg);
    X = X_out.str();
    Y = Y_out.str();
    cv::putText(frame_, color + "(" + X + "," + Y + ")", Point(obj_.x, obj_.y), 0, 0.5, Textcolor, 1);

    //==========球門射擊點 左右邊界繪製==============
    if (color == "B" || color == "Y")
    {
        circle(frame_, Point(obj_.right_x, obj_.right_y), 2, Scalar(0, 0, 0), -1);
        circle(frame_, Point(obj_.left_x, obj_.left_y), 2, Scalar(0, 0, 0), -1);
        circle(frame_, Point(obj_.fix_x, obj_.fix_y), 4, Scalar(0, 255, 0), -1);
    }
}
