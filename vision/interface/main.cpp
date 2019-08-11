#include "interface.h"

void SigintHandler(int sig)
{
    cv::destroyAllWindows();
    ROS_INFO("shutting down!");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface", ros::init_options::NoSigintHandler);
    ros::NodeHandle h_node;
    signal(SIGINT, SigintHandler);
    //Vision cam(VISION_TOPIC);
    Vision cam("camera/image_raw");
    ros::Rate loop_rate(200);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Node exit");
    printf("Process exit\n");
    return 0;
}
