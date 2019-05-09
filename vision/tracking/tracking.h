#include "nodehandle.h"
#define REDITEM 0x01
#define GREENITEM 0x02
#define BLUEITEM 0x04
#define YELLOWITEM 0x08
#define WHITEITEM 0x10 
#define VISION_TOPIC "/camera/image_raw"

class Vision: NodeHandle
{
public:
	Vision();
	Vision(string topic);
	~Vision();
	void release();
	
private:
	ros::NodeHandle nh;
	ros::Subscriber image_sub;
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	//void imageCb(const sensor_msgs::CompressedImageConstPtr& msg);
	double Rate();
    double FrameRate;
    bool init;
    DetectedObject b1,b2,b3,b4;
    Mat Monitor;
    void ObjectProcessing();
    void objectdet_change( int color, DetectedObject &obj_item);
    void object_compare(DetectedObject &FIND_Item, int distance, int angle);
    void find_around(Mat &frame_, deque<int> &find_point, int distance, int angle, int &size, int color);
    void Mark_point(Mat &frame_, deque<int> &find_point, int distance, int angle, int x, int y, int &size, int color);
    void find_object_point(DetectedObject &obj_, int color);
    void draw_Line(Mat &frame_, int obj_distance_max, int obj_distance_min, int obj_angle);
    void draw_ellipse(Mat &frame_, DetectedObject &obj_, int color);
    void draw_point(cv::Mat &frame_, DetectedObject &obj_, string color, Scalar Textcolor);
//==========================================
    cv::Mat Source;
};

