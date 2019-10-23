#include <cstdio>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "imu_3d/inertia.h"
#include "self_localization/resetParticles.h"
#include "self_localization/setAugmentParam.h"
#include "self_localization/setCmpsWeight.h"
#include "mcl.h"

using namespace std;
using namespace cv;

class Localization
{
public:
    Localization();
    ~Localization();
    void draw_particles();
private:
    MCL mcl;
    Mat field_map;
    Mat particles_map;
    vector<int> point;
    vector<int> good_point;
    vector<int> bad_point;
    double robot_x;
    double robot_y;
    double robot_w;
    ros::NodeHandle nh;
    ros::Subscriber sensor_sub;
    ros::Subscriber vel_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber imu_reset_sub;
    ros::Subscriber reset_sub;
    ros::Subscriber aug_sub;
    ros::Subscriber weight_sub;
    ros::Subscriber save_sub;
    ros::Publisher pos_pub;
    ros::Publisher sd_pub;
    ros::Publisher image_pub;
    double imu_angle_offset;
    double imu_angle;
    void sensorCallback(const std_msgs::Int32MultiArray msg);
    void resetCallback(const self_localization::resetParticles msg);
    void augCallback(const self_localization::setAugmentParam msg);
    void weightCallback(const self_localization::setCmpsWeight msg);
    void saveCallback(const std_msgs::Empty msg);
    void velCallback(const geometry_msgs::Twist msg);
    void imuCallback(const imu_3d::inertia msg);
    void imuresetCallback(const std_msgs::Int32 msg);
    void pos_publisher(int x, int y, double w);
    void image_publisher(Mat image);
    void sd_publisher(double sd);
    void Readyaml();
    void Saveyaml();
    void get_parameter();
    void draw_field();
};
