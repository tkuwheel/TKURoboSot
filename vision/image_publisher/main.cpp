#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <signal.h>

#define TEST ros::package::getPath("vision")+"/image_publisher/image.png"

using namespace cv;
using namespace std;

void SigintHandler(int sig)
{
    ROS_INFO("shutting down!");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle h_node;
    signal(SIGINT, SigintHandler);

    Mat img = imread(TEST, CV_LOAD_IMAGE_COLOR);

    ros::Publisher img_pub = h_node.advertise<sensor_msgs::Image>("/camera/image_raw", 1);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

    ros::Rate loop_rate(60); //Image transfer speed(hz)
    while (ros::ok())
    {
        img_pub.publish(msg);
        if (!img.empty())
        {
            //cout<<img.size()<<endl;
            //imshow("Publisher window", img);
            //waitKey(10);
        }
        else
        {
            cout << "No image detected\n";
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Node exit");
    printf("Process exit\n");
    return 0;
}
