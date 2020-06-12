#include "nodehandle.h"

class Vision : protected NodeHandle
{
  public:
    Vision();
    Vision(string topic);
    ~Vision();
    cv::Mat GetSource() { return Source; }
    cv::Mat GetMonitor() { return Monitor; }

  private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    //void imageCb(const sensor_msgs::CompressedImageConstPtr& msg);
    double Rate();
    double FrameRate;
    void ObjectProcessing();
    void objectdet_change(int color, DetectedObject &obj_item);
    void object_compare(DetectedObject &FIND_Item, int distance, int angle);
    void find_around(Mat &frame_, deque<int> &find_point, int distance, int angle, int &size, int color);
    void find_around_black(Mat &frame_, deque<int> &find_point, int distance, int angle, int &size, int color);
    void Mark_point(Mat &frame_, deque<int> &find_point, int distance, int angle, int x, int y, int &size, int color);
    void find_object_point(DetectedObject &obj_, int color);
    void find_edge_point(DetectedObject &obj_, int color);
    void find_shoot_point(DetectedObject &obj_, int color);
    void draw_center();
    void draw_ellipse(Mat &frame_, DetectedObject &obj_, int color);
    void draw_ellipse2(Mat &frame_, DetectedObject &obj_, int color);
    void draw_Line(Mat &frame_, int obj_distance_max, int obj_distance_min, int obj_angle, int color);
    void draw_point(cv::Mat &frame_, DetectedObject &obj_, string color, Scalar Textcolor);
    void source2threshold();
    cv::Mat Monitor;
    cv::Mat Source;
    cv::Mat Threshold;
};
