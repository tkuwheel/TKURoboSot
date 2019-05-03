#include "nodehandle.h"

class Vision : protected NodeHandle
{
  public:
    Vision();
    Vision(string topic);
    ~Vision();
    void release();
    cv::Mat Black_Item(const cv::Mat iframe);

  private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    //void imageCb(const sensor_msgs::CompressedImageConstPtr& msg);
    //NodeHandle Node;
    double Rate();
    double FrameRate;
    //==========================================
    cv::Mat Source;
};
