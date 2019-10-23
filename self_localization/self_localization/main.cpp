#include "src/localization.h"

int main(int argc,char **argv){
    ros::init(argc, argv, "self_localization");
    ros::NodeHandle nh;

    Localization localization;
    ros::Rate loop_rate(30);
    while(ros::ok()){
        localization.draw_particles();
        ros::spinOnce();
		loop_rate.sleep();
    }
    ROS_INFO("Node exit");
    return 0;
}

