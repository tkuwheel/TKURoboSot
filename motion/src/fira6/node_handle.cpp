#include "node_handle.h"
Motion_nodeHandle::Motion_nodeHandle(int argc, char** argv)
{
    this->robotCMD = {0, 0, 0, 0, 0};
    robotCMD.hold_ball = true;
    this->motion_flag = false;
    this->remote = false;
    this->holdBall = true;
#ifdef DEBUG
    std::cout << "Motion_nodeHandle(DEBUG)\n";
    std::cout << "x_speed: " << this->robotCMD.x << std::endl;
    std::cout << "y_speed: " << this->robotCMD.y << std::endl;
    std::cout << "yaw_speed: " << this->robotCMD.yaw << std::endl;
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
    int p = pthread_create(&tid, NULL, (THREADFUNCPTR)&Motion_nodeHandle::mpThreadRun, this);
    if(p != 0){
        printf("node thread error\n");
        exit(EXIT_FAILURE);
    }
}

void* Motion_nodeHandle::mpThreadRun(void* p)
{
    ((Motion_nodeHandle*)p)->mRun();
    pthread_exit(NULL);
}

void Motion_nodeHandle::mRun()
{
    while(ros::ok()){
        ros::spin();
    }
}

void Motion_nodeHandle::motionCallback(const geometry_msgs::Twist::ConstPtr &motion_msg)
{
    this->robotCMD.x = motion_msg->linear.x;
    this->robotCMD.y = motion_msg->linear.y;
    this->robotCMD.yaw = motion_msg->angular.z;
    this->motion_flag = true;
#ifdef DEBUG
    std::cout << "motionCallback(DEBUG)\n";
    std::cout << std::dec;
    std::cout << "X axis speed(%): " << this->robotCMD.x << std::endl;
    std::cout << "Y axis speed(%): " << this->robotCMD.y << std::endl;
    std::cout << "yaw speed(%): " << this->robotCMD.yaw << std::endl;
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

RobotCommand Motion_nodeHandle::getMotion()
{
    return robotCMD;
}

void Motion_nodeHandle::pub_robotFB(RobotCommand robotFB)
{
    geometry_msgs::Twist FB;

    FB.linear.x = robotFB.x;
    FB.linear.y = robotFB.y;
    FB.angular.z = robotFB.yaw;
    pub(FB);
}

void Motion_nodeHandle::clearShoot()
{
    robotCMD.shoot_power = 0;
}

int Motion_nodeHandle::clearAll()
{
    robotCMD.x = 0;
    robotCMD.y = 0;
    robotCMD.yaw = 0;
    robotCMD.shoot_power = 0;
//    robotCMD.hold_ball = true;
}

bool Motion_nodeHandle::getMotionFlag()
{
    if(motion_flag){
        motion_flag = false;
        return true;
    }else{
        return false;
    }
}

void Motion_nodeHandle::ShowCommand()
{
    printf("x speed %f\n", robotCMD.x);
    printf("y speed %f\n", robotCMD.y);
    printf("yaw speed %f\n", robotCMD.yaw);
    printf("shoot power %d\n", robotCMD.shoot_power);
}
