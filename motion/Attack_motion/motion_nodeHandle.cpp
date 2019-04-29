#include "motion_nodeHandle.h"
Motion_nodeHandle::Motion_nodeHandle(int argc, char** argv)
{
    this->robotCMD = {0, 0, 0, 0, 0};
    this->RX = {0, 0, 0, 0, 0, 0, 0};
    this->motion_flag = false;
//    this->node_robotCMD = new robot_command;
//    this->node_robotCMD->x_speed = new double;
//    this->node_robotCMD->y_speed = new double;
//    this->node_robotCMD->yaw_speed = new double;
//    this->node_robotCMD->shoot_power = new int;
//    this->node_robotCMD->hold_ball = new unsigned char;
//    this->node_RX = new serial_rx;
//    std::memset(this->node_robotCMD->x_speed, 0, sizeof(double));
//    std::memset(this->node_robotCMD->y_speed, 0, sizeof(double));
//    std::memset(this->node_robotCMD->yaw_speed, 0, sizeof(double));
//    std::memset(this->node_robotCMD->shoot_power, 0, sizeof(int));
//    std::memset(this->node_robotCMD->hold_ball, 0, sizeof(unsigned char));
    this->remote = false;
    this->holdBall = false;
#ifdef DEBUG
    std::cout << "Motion_nodeHandle(DEBUG)\n";
    std::cout << "x_speed: " << this->robotCMD.x_speed << std::endl;
    std::cout << "y_speed: " << this->robotCMD.y_speed << std::endl;
    std::cout << "yaw_speed: " << this->robotCMD.yaw_speed << std::endl;
    std::cout << "shoot_power: " << this->robotCMD.shoot_power << std::endl;
#endif
    init(argc, argv);
}

Motion_nodeHandle::~Motion_nodeHandle()
{
    ros::shutdown();
#ifdef DEBUG
    std::cout << "~Motion_nodeHandle(DEBUG)\n";
#endif
}

void Motion_nodeHandle::init(int argc, char **argv)
{
    std::cout << "==== Init node ====\n";
    ros::init(argc, argv, "Attack_motion");
#ifdef DEBUG
    std::cout << "nodeHandle init(DEBUG)\n";
    std::cout << "PATH= " << *argv << std::endl;
#endif
    this->n = new ros::NodeHandle();

    motionFB_pub = n->advertise<geometry_msgs::Twist>(motion_feedback_topic_name,1000);
    motion_sub = n->subscribe<geometry_msgs::Twist>(motion_topic_name, 1000, &Motion_nodeHandle::motionCallback, this);
    shoot_sub = n->subscribe<std_msgs::Int32>(shoot_topic_name, 1000, &Motion_nodeHandle::shootCallback, this);
    remote_sub = n->subscribe<std_msgs::Bool>(remote_topic_name, 1000, &Motion_nodeHandle::remoteCallback, this);
    holdBall_sub = n->subscribe<std_msgs::Bool>(holdBall_topic_name, 1000, &Motion_nodeHandle::holdBallCallback, this);
    int p = 0;
    p = pthread_create(&tid, NULL, (THREADFUNCPTR)&Motion_nodeHandle::pThreadRun, this);
    if(p != 0){
        printf("motion thread error\n");
        exit(EXIT_FAILURE);
    }
}

void Motion_nodeHandle::motionCallback(const geometry_msgs::Twist::ConstPtr &motion_msg)
{
    this->robotCMD.x_speed = motion_msg->linear.x;
    this->robotCMD.y_speed = motion_msg->linear.y;
    this->robotCMD.yaw_speed = motion_msg->angular.z;
    this->motion_flag = true;
#ifdef DEBUG
    std::cout << "motionCallback(DEBUG)\n";
    std::cout << std::dec;
    std::cout << "X axis speed(%): " << this->robotCMD.x_speed << std::endl;
    std::cout << "Y axis speed(%): " << this->robotCMD.y_speed << std::endl;
    std::cout << "yaw speed(%): " << this->robotCMD.yaw_speed << std::endl;
    std::cout << std::endl;
#endif
}

void Motion_nodeHandle::shootCallback(const std_msgs::Int32::ConstPtr &shoot_msg)
{
    this->robotCMD.shoot_power = shoot_msg->data;
    this->motion_flag = true;
    
#ifdef DEBUG
    std::cout << "shootCallback(DEBUG)\n";
    std::cout << std::dec;
    std::cout << "shoot power(%): " << robotCMD.shoot_power << std::endl;
    std::cout << std::endl;
#endif
}

void Motion_nodeHandle::remoteCallback(const std_msgs::Bool::ConstPtr &remote_msg)
{
    this->remote = remote_msg->data;
    this->robotCMD.remote = remote_msg->data;
    this->motion_flag = true;
#ifdef DEBUG
    std::cout << "remoteCallback(DEBUG)\n";
    //std::cout << std::dec;
    std::cout << "remote : " << this->remote << std::endl;
    std::cout << std::endl;

#endif
}

void Motion_nodeHandle::holdBallCallback(const std_msgs::Bool::ConstPtr &holdBall_msg)
{
    this->robotCMD.hold_ball = holdBall_msg->data;
    this->motion_flag = true;
}

void Motion_nodeHandle::pub(const geometry_msgs::Twist &pub_msgs)
{
    motionFB_pub.publish(pub_msgs);
}

void Motion_nodeHandle::run()
{

    while(ros::ok()){
        
        ros::spin();
    }
}

void* Motion_nodeHandle::pThreadRun(void* p)
{
    Motion_nodeHandle* Node = (Motion_nodeHandle*)p;
    Node->run();
    pthread_exit(NULL);
}

robot_command Motion_nodeHandle::getMotion()
{
    this->motion_flag = false;
//    robot_command CMD = this->robotCMD;
//    this->robotCMD = {0, 0, 0, 0, 0, 0};
    return robotCMD;
}

void Motion_nodeHandle::pub_robotFB(robot_command &robotFB)
{
    geometry_msgs::Twist FB;

    FB.linear.x = robotFB.x_speed;
    FB.linear.y = robotFB.y_speed;
    FB.angular.z = robotFB.yaw_speed;
    pub(FB);
}

void Motion_nodeHandle::clear()
{
    if(this->remote == false){
        this->robotCMD.x_speed = 0;
        this->robotCMD.y_speed = 0;
        this->robotCMD.yaw_speed = 0;
    }
    this->robotCMD.shoot_power = 0;
}

bool Motion_nodeHandle::getMotionFlag()
{
    return this->motion_flag;
}
