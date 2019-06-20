#include "localization.h"
#include <ros/package.h>
#include <fstream>
#include <iomanip>

#define N_PARTICLE 600
#define TO_RAD M_PI/180.0
#define TO_DEG 180.0/M_PI

#define FIELD_WIDTH 600
#define FIELD_HEIGHT 400

#define XLINE1 300
#define XLINE2 XLINE1-40
#define XLINE3 XLINE1-75
#define XLINE4 0
#define XLINE5 -(XLINE3)
#define XLINE6 -(XLINE2)
#define XLINE7 -(XLINE1)
#define XLINE8 75
#define XLINE9 -XLINE8

#define YLINE1 200
#define YLINE2 100
#define YLINE3 80
#define YLINE4 -(YLINE3)
#define YLINE5 -(YLINE2)
#define YLINE6 -(YLINE1)

#define CENTER_RADIUS 60
#define CENTER_RADIUS2 25

#define DISTANCE_MATRIX_WIDTH 751
#define DISTANCE_MATRIX_HEIGHT 551

#define YAML_PATH ros::package::getPath("self_localization")+"/config/localization.yaml"
#define STRATEGY_PATH ros::package::getPath("self_localization")+"/config/strategy.yaml"
Localization::Localization()
{
    //Readyaml();
    sensor_sub = nh.subscribe("/vision/mcl/WhiteRealDis", 1000, &Localization::sensorCallback, this);
    reset_sub = nh.subscribe("/mcl/resetParticles", 1000, &Localization::resetCallback, this);
    aug_sub = nh.subscribe("/mcl/setAugmentParam", 1000, &Localization::augCallback, this);
    weight_sub = nh.subscribe("/mcl/setCmpsWeight", 1000, &Localization::weightCallback, this);
    save_sub = nh.subscribe("/mcl/save", 1000, &Localization::saveCallback, this);
    vel_sub = nh.subscribe("/motion/motionFB", 1000, &Localization::velCallback, this);
    imu_sub = nh.subscribe("/imu_3d", 1000, &Localization::imuCallback, this);
    imu_reset_sub = nh.subscribe("/mcl/imu_reset", 1000, &Localization::imuresetCallback, this);
    pos_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/akf_pose", 1);
    sd_pub = nh.advertise<std_msgs::Float32>("/mcl/std", 1);
    image_pub = nh.advertise<sensor_msgs::Image>("/mcl/image", 1);
    draw_field();
    imu_angle_offset = 0;
    imu_angle = 0;
}
Localization::~Localization()
{
    destroyAllWindows();
}
void Localization::Readyaml()
{
    std::string param = YAML_PATH;
    const char *parampath = param.c_str();
    if (ifstream(parampath))
    {
        std::string temp = "rosparam load " + param + " /mcl";
        const char *load = temp.c_str();
        system(load);
        cout << "Read the yaml file" << endl;
    }
    else
    {
        cout << "yaml file does not exist" << endl;
    }
    //get_parameter();
}
void Localization::Saveyaml()
{
    std::string param = YAML_PATH;
    std::string temp = "rosparam dump " + param + " /FIRA";
    const char *save = temp.c_str();
    system(save);

    cout << "Save the yaml file" << endl;
    get_parameter();
}
void Localization::get_parameter()
{
    cout << "get parameter" << endl;
    double a_fast,a_slow,wcmps;
    //===================FPS參數==========================
    nh.getParam("/FIRA/mcl/a_fast", a_fast);
    nh.getParam("/FIRA/mcl/a_slow", a_slow);
    nh.getParam("/FIRA/mcl/wcmps", wcmps);
    mcl.setAugmentParam(a_fast, a_slow);
    mcl.setCmpsWeight(wcmps);
}
void Localization::sensorCallback(const std_msgs::Int32MultiArray msg)
{
    point.clear();
    point = msg.data;
    std::vector<MCL::SensorData> mcl_sensor_data;
    for(int i = 0; i<point.size()/2;i+=2){
        mcl_sensor_data.push_back(MCL::SensorData(point[i],point[i+1]));
    }
    if(mcl_sensor_data.size()>2){
        mcl.updateSensor(mcl_sensor_data);
    }
    //mcl.updateMotion(0,0,0);
    
}
void Localization::resetCallback(const self_localization::resetParticles msg)
{
    cout<<"resetParticles"<<endl;
    //cout<< msg.x<<"  "<< msg.y<<"  "<< msg.w<<endl;
    mcl.resetParticles(msg.init, msg.x, msg.y, msg.w);
}
void Localization::augCallback(const self_localization::setAugmentParam msg)
{
    cout<<"setAugmentParam"<<endl;
    mcl.setAugmentParam(msg.a_fast, msg.a_slow);
}
void Localization::weightCallback(const self_localization::setCmpsWeight msg)
{
    cout<<"setCmpsWeight"<<endl;
    mcl.setCmpsWeight(msg.wcmps);
}
void Localization::saveCallback(const std_msgs::Empty msg)
{
    Saveyaml();
}
void Localization::velCallback(const geometry_msgs::Twist msg)
{
    static long double x=0;
    static long double y=0;
    static long double w=0;

    double ALPHA = 0.5;
    double dt;
    static int frame_counter = 0;
    static double frame_rate = 0.0;
    static double StartTime = ros::Time::now().toNSec();
    double EndTime;
    bool filter = false;
    frame_counter++;
    if (frame_counter == 2)
    {
        EndTime = ros::Time::now().toNSec();
        dt = (EndTime - StartTime)/1000000000;
        //cout<<dt<<endl;
        StartTime = EndTime;
        if (dt != 0)
        {
            static long double vx=0,vy=0,vw=0;
            double vx_,vy_,vw_;
            vx_ = 3*(msg.linear.x-x)/dt;
            vy_ = 3*(msg.linear.y-y)/dt;
            vw_ = 3*(msg.angular.z*TO_RAD-w)/dt;
            //vw = 2*(imu_angle*TO_RAD-w)/dt;
            //cout<<"================="<<endl;
            //cout<<vx_<<endl;
            //cout<<vy_<<endl;
            //cout<<vw_<<endl;
            //cout<<"================="<<endl;
            if(abs(vx_)>5){
                filter = true;
                //vx=0;
                //cout<<"fucck1"<<endl;
            }
            if(abs(vy_)>5){
                filter = true;
                //vy=0;
                //cout<<"fucck2"<<endl;
            }
            if(abs(vw-vw_)>8){
                filter = true;
                //vw=0;
                //cout<<"fucck3"<<endl;
 
            }
             //vx=0,vy=0,vw=0;
            
            if(!filter){
                vx=vx_;
                vy=vy_;
                vw=vw_;
            }
            if(fabs(vx_)<0.4||fabs(vy_)<0.4){
                vx=vx_;
                vy=vy_;
            }
            if(fabs(vw_)<5){
                vw=vw_;
            }
            if(abs(vx)>8||abs(vy)>8||abs(vw)>20){
                //cout<<"vw>30";
                vx=0;
                vy=0;
                vw=0;
                cout<<"speed filter on\n";
            }
            // cout<<"================="<<endl;
            //cout<<vx<<endl;
            //cout<<vy<<endl;
            //cout<<vw<<endl;
            //if(vx!=0&&vy!=0&&vw!=0){
                //cout<<"vx,vy,vw = "<<vx<<" "<<vy<<" "<<vw<<endl;
            //}
            mcl.updateMotion(vy,vx,vw);
            //mcl.updateMotion(0,0,0);
            x=msg.linear.x;
            y=msg.linear.y;
            w=msg.angular.z*TO_RAD;
            //w=imu_angle*TO_RAD;
            //frame_rate = (1000000000.0 / dt) * ALPHA + frame_rate * (1.0 - ALPHA);
            //cout << "FPS: " << frame_rate << endl;
        }

        frame_counter = 0;
    }
}
void Localization::imuCallback(const imu_3d::inertia msg)
{
    imu_angle = msg.yaw*360/(M_PI*2);
    imu_angle=(360-imu_angle)-imu_angle_offset+90;
    
    if(imu_angle<0)imu_angle = 360+imu_angle;
    if(imu_angle>360)imu_angle = imu_angle-360;
    //cout<<imu_angle<<endl;
    
    //==========
    mcl.updateCompass(imu_angle);
    //==========
}
void Localization::imuresetCallback(const std_msgs::Int32 msg)
{
    imu_angle_offset = imu_angle-msg.data;
}
void Localization::draw_field()
{
    Mat self_map(DISTANCE_MATRIX_HEIGHT, DISTANCE_MATRIX_WIDTH, CV_8UC3, Scalar(0,150,0));
    line(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE1), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE1), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE6), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE6), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE1), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE6), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE1), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE6), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE4,self_map.rows/2+YLINE1), Point(self_map.cols/2+XLINE4,self_map.rows/2+YLINE6), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE2,self_map.rows/2+YLINE3), Point(self_map.cols/2+XLINE2,self_map.rows/2+YLINE4), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE6,self_map.rows/2+YLINE3), Point(self_map.cols/2+XLINE6,self_map.rows/2+YLINE4), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE3,self_map.rows/2+YLINE2), Point(self_map.cols/2+XLINE3,self_map.rows/2+YLINE5), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2+YLINE2), Point(self_map.cols/2+XLINE5,self_map.rows/2+YLINE5), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE2,self_map.rows/2+YLINE3), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE3), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE2,self_map.rows/2+YLINE4), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE4), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE3,self_map.rows/2+YLINE2), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE2), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE3,self_map.rows/2+YLINE5), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE5), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE6,self_map.rows/2+YLINE3), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE3), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE6,self_map.rows/2+YLINE4), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE4), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2+YLINE2), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE2), Scalar(255,255,255), 3);
    line(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2+YLINE5), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE5), Scalar(255,255,255), 3);
    circle(self_map, Point(self_map.cols/2,self_map.rows/2), CENTER_RADIUS, Scalar(255, 255, 255), 3);
    ellipse(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE1), Size(CENTER_RADIUS2,CENTER_RADIUS2), 0, 180, 270, Scalar(255,255,255), 3);
    ellipse(self_map, Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE1), Size(CENTER_RADIUS2,CENTER_RADIUS2), 0, 270, 360, Scalar(255,255,255), 3);
    ellipse(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE6), Size(CENTER_RADIUS2,CENTER_RADIUS2), 0, 90, 180, Scalar(255,255,255), 3);
    ellipse(self_map, Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE6), Size(CENTER_RADIUS2,CENTER_RADIUS2), 0, 0, 90, Scalar(255,255,255), 3);

    ellipse(self_map, Point(self_map.cols/2+XLINE3,self_map.rows/2), Size(CENTER_RADIUS,CENTER_RADIUS), 0, 90, 270, Scalar(255,255,255), 3);
    ellipse(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2), Size(CENTER_RADIUS,CENTER_RADIUS), 0, 0, 90, Scalar(255,255,255), 3);
    ellipse(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2), Size(CENTER_RADIUS,CENTER_RADIUS), 0, 270, 360, Scalar(255,255,255), 3);

    circle(self_map, Point(self_map.cols/2+XLINE8,self_map.rows/2+YLINE2), 3, Scalar(255, 255, 255), -1);
    circle(self_map, Point(self_map.cols/2+XLINE8,self_map.rows/2+YLINE5), 3, Scalar(255, 255, 255), -1);
    circle(self_map, Point(self_map.cols/2+XLINE9,self_map.rows/2+YLINE2), 3, Scalar(255, 255, 255), -1);
    circle(self_map, Point(self_map.cols/2+XLINE9,self_map.rows/2+YLINE5), 3, Scalar(255, 255, 255), -1);

    rectangle(self_map, Point(self_map.cols/2+XLINE7-40,self_map.rows/2+YLINE4), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE3), Scalar(0,255,255), -1);
    rectangle(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE4), Point(self_map.cols/2+XLINE1+40,self_map.rows/2+YLINE3), Scalar(255,0,0), -1);

    field_map=self_map.clone();
}
void Localization::draw_particles()
{
    particles_map=field_map.clone();
    auto particles = mcl.getParticles();
    for(auto p : particles)
    {
        int x_,y_,w_,x1,y1;
        x1=MCL::x(p)+particles_map.cols/2;
        y1=MCL::y(p)+particles_map.rows/2;
        circle(particles_map, Point(MCL::x(p)+particles_map.cols/2,MCL::y(p)+particles_map.rows/2), 3, Scalar(0, 0, 0), 1);
        x_ = x1+5 * cos(MCL::w(p) * TO_RAD);
        y_ = y1-5 * sin(MCL::w(p) * TO_RAD);
        line(particles_map, Point(MCL::x(p)+particles_map.cols/2, MCL::y(p)+particles_map.rows/2), Point(x_, y_), Scalar(0, 0, 0), 1);
    }

    int x,y,w,x_,y_;

//=================================================================
    auto weighted_estimation = mcl.weighted_estimation();
    x=std::get<0>(weighted_estimation);
    y=std::get<1>(weighted_estimation);
    w=std::get<2>(weighted_estimation);

    //cout<<"weighted_estimation:("<<x<<", "<<y<<") angle:"<<w<<endl;
    x+=particles_map.cols/2;
    y+=particles_map.rows/2;
    x_= x+20 * cos(w * TO_RAD);
    y_= y-20 * sin(w * TO_RAD);
    line(particles_map, Point(x, y), Point(x_, y_), Scalar(255, 255, 255), 2);
    circle(particles_map, Point(x, y), 20, Scalar(255, 255, 255), 2);
//=================================================================
    //imu angle line
    x_= x+20 * cos(imu_angle* TO_RAD);
    y_= y-20 * sin(imu_angle * TO_RAD);
    line(particles_map, Point(x, y), Point(x_, y_), Scalar(0, 0, 255), 2);
//=================================================================
    auto estimation = mcl.estimation();
    x=std::get<0>(estimation);
    y=std::get<1>(estimation);
    w=std::get<2>(estimation);
    //cout<<mcl.get_sd();
    sd_publisher(mcl.get_sd());
    //==========
    //mcl.updateCompass(w);
    //cout<<w<<endl;
    //==========
    //cout<<"("<< std::right<<std::setw(4)<<x<<" ,"<<std::setw(4)<<y<<")   "<<"angle: "<< std::left<<std::setw(4)<<w<<"  sd_xy: "<<mcl.get_sd()<<endl;

    //cout<<"estimation:("<<x<<", "<<y<<") angle:"<<w<<endl;
    x_= x+20 * cos(w * TO_RAD);
    y_= y-20 * sin(w * TO_RAD);
    pos_publisher(x, -y, w);

    x+=particles_map.cols/2;
    y+=particles_map.rows/2;
    x_+=particles_map.cols/2;
    y_+=particles_map.rows/2;

    line(particles_map, Point(x, y), Point(x_, y_), Scalar(0, 255, 255), 2);
    circle(particles_map, Point(x, y), 20, Scalar(0, 255, 255), 2);
    circle(particles_map, Point(x_, y_), 5, Scalar(0, 255, 255), -1);
    circle(particles_map, Point(x_, y_), 5, Scalar(0, 0, 0), 1);
//=================================================================
    x_= x+20 * cos(imu_angle* TO_RAD);
    y_= y-20 * sin(imu_angle * TO_RAD);
    line(particles_map, Point(x, y), Point(x_, y_), Scalar(0, 0, 255), 2);
//=================================================================

    for(int i = 0; i<point.size();i+=2){
        int xl=point[i];
        int yl=point[i+1];
        int x2;
        int y2;
        double angle = atan2(yl, xl);
        double distance = sqrt(xl*xl+yl*yl);
        x2=x+distance*cos(-angle+(w * TO_RAD));
        y2=y-distance*sin(-angle+(w * TO_RAD));
        circle(particles_map, Point(x2, y2), 3, Scalar(255, 0, 0), -1);
    }

    image_publisher(particles_map);
    //imshow("particles_map",particles_map);
    waitKey(10);
}
void Localization::pos_publisher(int x,int y, double w)
{

    double w_=w*TO_RAD;
    double qx = sin(0)*cos(0)*cos(w_/2)-cos(0)*sin(0)*sin(w_/2);
    double qy = cos(0)*sin(0)*cos(w_/2)+sin(0)*cos(0)*sin(w_/2);
    double qz = cos(0)*cos(0)*sin(w_/2)-sin(0)*sin(0)*cos(w_/2);
    double qw = cos(0)*cos(0)*cos(w_/2)+sin(0)*sin(0)*sin(w_/2);

    //double angle = atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)/M_PI*180;
    //if(angle>=360)angle-=360;
    //if(angle<0)angle+=360;
    //cout<<setw(5)<<w<<setw(10)<<angle<<endl;

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "MCL_pose";
    msg.pose.pose.position.x = ((double)x/100);
    msg.pose.pose.position.y = ((double)y/100);
    msg.pose.pose.orientation.x = qx;
    msg.pose.pose.orientation.y = qy;
    msg.pose.pose.orientation.z = qz;
    msg.pose.pose.orientation.w = qw;
    pos_pub.publish(msg);
}
void Localization::sd_publisher(double sd)
{
    //cout<<mcl.get_sd()<<endl;
    std_msgs::Float32 msg;
    msg.data=sd;
    sd_pub.publish(msg);
}
void Localization::image_publisher(Mat image)
{
    sensor_msgs::ImagePtr frameMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_pub.publish(frameMsg);
}
