#include "tracking.h"
#include "omp.h"

Vision::Vision():
    FrameRate(0.0),
    init(true)
{
    image_sub = nh.subscribe(VISION_TOPIC, 1,&Vision::imageCb,this);
}
Vision::Vision(string topic):
    FrameRate(0.0),
    init(true)
{
	image_sub = nh.subscribe(topic, 1,&Vision::imageCb,this);

}
Vision::~Vision(){
	Source.release();
	destroyAllWindows(); 
}
double Vision::Rate()
{
    double ALPHA = 0.5;
    double dt;
    static int frame_counter = 0;
    static double frame_rate = 0.0;
    static double StartTime = ros::Time::now().toSec();
    double EndTime;

    frame_counter++;
    if (frame_counter == 10)
    {
        EndTime = ros::Time::now().toSec();
        dt = (EndTime - StartTime) / frame_counter;
        StartTime = EndTime;
        if (dt != 0)
        {
            frame_rate = (1.0 / dt) * ALPHA + frame_rate * (1.0 - ALPHA);
            //cout << "FPS: " << frame_rate << endl;
        }

        frame_counter = 0;
    }
    return frame_rate;
}
//=============================影像接收=======================================
void Vision::imageCb(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //convert image data
		if(!cv_ptr->image.empty()){
            Source = cv_ptr->image.clone();
		    cv::flip(Source, Source, 1); // mirror image adjustment
            Monitor = Source.clone();
			//cv::imshow("Image window", cv_ptr->image);
            //cv::imshow("Source", Source);
			//cv::waitKey(10);
			FrameRate = Rate();
            ObjectProcessing();
            
		}else{
            cout<<"Image empty"<<endl;
        }
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert to image!");
		return;
	}
}

//==========================================================================
void Vision::ObjectProcessing()
{
    b1.Reset();
    b2.Reset();
    b3.Reset();
    //b4.Reset();
    if(init){
        DetectedObject t1,t2,t3,t4;
        objectdet_change(REDITEM, t1);
        objectdet_change(REDITEM, t2);
        objectdet_change(REDITEM, t3);
        //objectdet_change(REDITEM, t4);
        DetectedObject temp[3]={t1,t2,t3};
        int length = sizeof(temp)/sizeof(temp[0]);
        for (int i = length - 1; i > 0; --i){
            for (int j = 0; j < i; ++j){
                if (temp[j].ang_min > temp[j + 1].ang_min){
                    swap(temp[j], temp[j + 1]);
                }
            }
        }
        b1=temp[0];
        b2=temp[1];
        b3=temp[2];
        //b4=temp[3];

        init = false;
    }else{
        objectdet_change(REDITEM, b1);
        objectdet_change(REDITEM, b2);
        objectdet_change(REDITEM, b3);
        
        //objectdet_change(REDITEM, b4);
    }
    draw_shotpoint();
    //Mat rot_mat = getRotationMatrix2D(Point(Monitor.cols/2,Monitor.rows/2), 180, 1);
    //warpAffine(Monitor, Monitor, rot_mat, Monitor.size());
    imshow("Monitor", Monitor);
    waitKey(10);
}
void Vision::objectdet_change( int color, DetectedObject &obj_item)
{
    int x, y;
    int x_, y_;
    int object_size;
    int dis, ang;

    Mat frame_(Source.cols, Source.rows, CV_8UC3, Scalar(0, 0, 0));

    DetectedObject FIND_Item=obj_item;
    //DetectedObject FIND_Item;
    deque<int> find_point;

    int dis_start=Magn_Near_StartMsg;
    int dis_end=Magn_Far_EndMsg;
    int angle_start=0;
    int angle_end=360;
    int search_angle=10;
    int search_dis=20;
    
    if(!init){
        //dis_start   = Frame_Area((obj_item.dis_min - search_dis), Magn_Far_EndMsg);
        //dis_end     = Frame_Area((obj_item.dis_max + search_dis), Magn_Far_EndMsg);
        dis_start   = Frame_Area((obj_item.distance - search_dis), Magn_Far_EndMsg);
        dis_end     = Frame_Area((obj_item.distance + search_dis), Magn_Far_EndMsg);
        angle_start = (obj_item.angle - search_angle);
        angle_end   = (obj_item.angle + search_angle);
        //cout<<obj_item.angle - search_angle<<endl;
        //cout<<(obj_item.angle + search_angle)<<endl;
        //dis_start=Magn_Near_StartMsg;
        //dis_end=Magn_Far_EndMsg;
        //angle_start = (obj_item.ang_min - search_angle);
        //angle_end   = (obj_item.ang_max + search_angle);
        //angle_start=0;
        //angle_end=360;
    }
    for (int distance = dis_start; distance <= dis_end; distance += Magn_Near_GapMsg)
    {
        for (int angle_ = angle_start; angle_ < angle_end; angle_ += Angle_Interval(distance))
        {
            int angle = Angle_Adjustment(angle_);
            //cout<<angle<<" "<<Angle_cos[angle]<<endl;
            if ((angle >= Unscaned_Angle[0] && angle <= Unscaned_Angle[1]) ||
                (angle >= Unscaned_Angle[2] && angle <= Unscaned_Angle[3]) ||
                (angle >= Unscaned_Angle[4] && angle <= Unscaned_Angle[5]) ||
                (angle >= Unscaned_Angle[6] && angle <= Unscaned_Angle[7]))
            {
                continue;
            }
            object_size = 0;
            FIND_Item.size = 0;
            //if(angle>360||angle<0)cout<<angle<<" "<<Angle_cos[angle]<<endl;
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

    int center_angle = Angle_Adjustment((obj_item.ang_max+obj_item.ang_min)/2);
    int center_dis = (obj_item.dis_max + obj_item.dis_min)/2;
    int center_x = CenterXMsg + (center_dis * Angle_cos[center_angle]);
    int center_y = CenterYMsg - (center_dis * Angle_sin[center_angle]);
    int center_r = (obj_item.dis_max - obj_item.dis_min)/2;
    circle(Source, Point(center_x, center_y), center_r, Scalar(255, 255, 255), -1);
    circle(Monitor, Point(center_x, center_y), center_r, Scalar(255, 255, 255), -1);

    int SizeFilter=20;
    //if (obj_item.size > SizeFilter)
    //{
        find_object_point(obj_item, color);
        if (obj_item.size > SizeFilter)
            draw_ellipse(Monitor, obj_item, color);
        if (color == REDITEM){
            draw_point(Monitor, obj_item, "R", Scalar(0, 0, 255));  
        }
    //}

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
            dis_f = distance + i;
            Magn_Near_GapMsg;

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
void Vision::Mark_point(Mat &frame_, deque<int> &find_point, int distance, int angle, int x, int y, int &size, int color)
{
    frame_.data[(y * frame_.cols + x) * 3 + 0] = 255;
    frame_.data[(y * frame_.cols + x) * 3 + 1] = 255;
    frame_.data[(y * frame_.cols + x) * 3 + 2] = 255;
    find_point.push_back(distance);
    find_point.push_back(angle);
    size += 1;
}
void Vision::find_object_point(DetectedObject &obj_, int color)
{
    int x, y;
    int x_, y_;

    int angle_, distance_;
    int find_angle;

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
        if(obj_.size>20){
            if(obj_.x==0){
                //cout<<"fuck"<<endl<<x<<endl;
            }
            obj_.x = x;
            obj_.y = y;
            obj_.distance = distance_;
            obj_.angle = find_angle;
        }
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
void Vision::draw_Line(Mat &frame_, int obj_distance_max, int obj_distance_min, int obj_angle)
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
}
void Vision::draw_ellipse(Mat &frame_, DetectedObject &obj_, int color)
{
    ellipse(frame_, Point(CenterXMsg, CenterYMsg), Size(obj_.dis_min, obj_.dis_min), 0, 360 - obj_.ang_max, 360 - obj_.ang_min, Scalar(255, 255, 0), 1);
    ellipse(frame_, Point(CenterXMsg, CenterYMsg), Size(obj_.dis_max, obj_.dis_max), 0, 360 - obj_.ang_max, 360 - obj_.ang_min, Scalar(255, 255, 0), 1);
    draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_max);
    draw_Line(frame_, obj_.dis_max, obj_.dis_min, obj_.ang_min);
    circle(frame_, Point(obj_.x, obj_.y), 2, Scalar(0, 0, 0), -1);
}
void Vision::draw_point(cv::Mat &frame_, DetectedObject &obj_, string color, Scalar Textcolor)
{
    std::string X;
    std::string Y;
    std::stringstream X_out;
    std::stringstream Y_out;

    //==========中心點繪製===============
    circle(frame_, Point(obj_.x, obj_.y), 5, Scalar(255, 0, 0), -1);
    X_out << obj_.x - CenterXMsg;
    Y_out << 0 - (obj_.y - CenterYMsg);
    X = X_out.str();
    Y = Y_out.str();
    //cv::putText(frame_, color + "(" + X + "," + Y + ")", Point(obj_.x, obj_.y), 0, 0.5, Textcolor, 1);

    cv::putText(frame_, "b1", Point(b1.x, b1.y), 0, 1, Textcolor, 2);
    cv::putText(frame_, "b2", Point(b2.x, b2.y), 0, 1, Textcolor, 2);
    cv::putText(frame_, "b3", Point(b3.x, b3.y), 0, 1, Textcolor, 2);
    //cv::putText(frame_, "b4", Point(b4.x, b4.y), 0, 1, Textcolor, 2);
    //cout<<b1.x<<endl<<b1.y<<endl;
}
void Vision::draw_shotpoint()
{
    int x, y;
    circle(Monitor, Point(CenterXMsg, CenterYMsg), 1, Scalar(0, 255, 0), 1);
    circle(Monitor, Point(CenterXMsg, CenterYMsg), InnerMsg, Scalar(0, 0, 255), 1);
    circle(Monitor, Point(CenterXMsg, CenterYMsg), OuterMsg, Scalar(0, 255, 0), 1);
    x = CenterXMsg + InnerMsg * cos(FrontMsg * PI / 180);
    y = CenterYMsg - InnerMsg * sin(FrontMsg * PI / 180);
    line(Monitor, Point(CenterXMsg, CenterYMsg), Point(x, y), Scalar(255, 0, 255), 1);

    int a1,a2,a3,a4;
    
    a1=atan2(-shot_pose[0].y-pose_y,shot_pose[0].x-pose_x)*180/PI-pose_w;
    a2=atan2(-shot_pose[1].y-pose_y,shot_pose[1].x-pose_x)*180/PI-pose_w;
    a3=atan2(-shot_pose[2].y-pose_y,shot_pose[2].x-pose_x)*180/PI-pose_w;
    a4=atan2(-shot_pose[3].y-pose_y,shot_pose[3].x-pose_x)*180/PI-pose_w;
    //cout<<a1<<endl;
    //cout<<FrontMsg<<endl;
    //cout<<FrontMsg+a1<<endl;
    x = CenterXMsg + OuterMsg * cos(Angle_Adjustment(FrontMsg+a1) * PI / 180);
    y = CenterYMsg - OuterMsg * sin(Angle_Adjustment(FrontMsg+a1) * PI / 180);
    line(Monitor, Point(CenterXMsg, CenterYMsg), Point(x, y), Scalar(0, 0, 255), 3);
    
    x = CenterXMsg + OuterMsg * cos(Angle_Adjustment(FrontMsg+a2) * PI / 180);
    y = CenterYMsg - OuterMsg * sin(Angle_Adjustment(FrontMsg+a2) * PI / 180);
    line(Monitor, Point(CenterXMsg, CenterYMsg), Point(x, y), Scalar(255, 0, 0), 3);

    x = CenterXMsg + OuterMsg * cos(Angle_Adjustment(FrontMsg+a3) * PI / 180);
    y = CenterYMsg - OuterMsg * sin(Angle_Adjustment(FrontMsg+a3) * PI / 180);
    line(Monitor, Point(CenterXMsg, CenterYMsg), Point(x, y), Scalar(255, 255, 255), 3);

    x = CenterXMsg + OuterMsg * cos(Angle_Adjustment(FrontMsg+a4) * PI / 180);
    y = CenterYMsg - OuterMsg * sin(Angle_Adjustment(FrontMsg+a4) * PI / 180);
    line(Monitor, Point(CenterXMsg, CenterYMsg), Point(x, y), Scalar(0, 255, 255), 3);
    
    
}
