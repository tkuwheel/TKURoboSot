#include "node_handle.h"
Motion_nodeHandle::Motion_nodeHandle(int argc, char** argv)
{
    this->robotCMD = {0};
    this->motion_flag = false;
    this->remote = false;
    this->holdBall = false;
    m_clear_flag = false;
    m_is_activated = false;
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
    int p = 0;
    p = pthread_create(&tid, NULL, (THREADFUNCPTR)&Motion_nodeHandle::pThreadRun, this);
    if(p != 0){
        printf("motion thread error\n");
        exit(EXIT_FAILURE);
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
    printf("shootCallback(DEBUG)\n");
    printf("shoot power(%%): %d\n\n", robotCMD.shoot_power);
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
    int frequency = 100;
    ros::Rate loop_rate(frequency);
    int counter_shoot = 0;
    int counter_command = 0;
    int counter = 0;
    int count = 0;
    while(ros::ok()){
        if(!m_is_activated){
            if(counter>=frequency){
                count++;
                counter = 0;
                printf("CANNOT GET COMMAND %d\r", count);
                fflush(stdout);
            }else{
                counter++;
            }
        }else{
            counter = 0;
        }
        if(robotCMD.shoot_power>0){
            counter_shoot++;
            if(counter_shoot>=(frequency/2)){
                clearShoot();
                counter_shoot = 0;
            }
        }else{
            counter_shoot = 0;
        }
        if(!motion_flag){
            if(counter_command>=frequency/2){
                clearAll();
                m_is_activated = false;
            }else{
                counter_command++;
                m_is_activated = true;
            }
        }else{
            counter_command = 0;
            m_is_activated = true;
        }
        if(m_clear_flag)motion_flag = false;
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    std::cout << "ROS shutdown\n";
}

void* Motion_nodeHandle::pThreadRun(void* p)
{
    ((Motion_nodeHandle*)p)->run();
    pthread_exit(NULL);
    return NULL;
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
//    robotCMD.hold_ball = false;
}

bool Motion_nodeHandle::getNodeFlag(bool &is_activated)
{
    m_clear_flag = true;
    is_activated = m_is_activated;
//    is_activated = true;
    if(m_is_activated){
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
