/*
 *   particle_filter.cpp
 *
 *   Date:2015
 *
 *   Author: Chien-Ming Lin
 *
 */
#include "particle_filter.hpp"
#define trust_imu false

Particle tmp_p;
bool reflash;

ParticleFilter::ParticleFilter(int p_Num, int sensor_LineNum, int map_W, int map_H)
{
    pNum = p_Num;
    sensorLineNum = sensor_LineNum;
    mapH=map_H;
    mapW=map_W;
}

void ParticleFilter::likeliHood_markWant(int wantIndex,double markNum){
    if( wantIndex> 0 && wantIndex < (mapW*mapH)){
        double tmp = markNum*255 - likelihood_map->data[wantIndex];
        if(tmp > 0){
            likelihood_map->data[wantIndex]=markNum*255;
            likeliHood_map[wantIndex] = markNum;
        }
    }
}

void ParticleFilter::likeliHood_surroundMark(int nowIndex,int dist,double markNum)
{
    likeliHood_markWant(nowIndex - dist*mapW -dist,markNum);
    likeliHood_markWant(nowIndex - dist*mapW      ,markNum);
    likeliHood_markWant(nowIndex - dist*mapW +dist,markNum);

    likeliHood_markWant(nowIndex             -dist,markNum);
    likeliHood_markWant(nowIndex                  ,markNum);
    likeliHood_markWant(nowIndex             +dist,markNum);

    likeliHood_markWant(nowIndex + dist*mapW -dist,markNum);
    likeliHood_markWant(nowIndex + dist*mapW      ,markNum);
    likeliHood_markWant(nowIndex + dist*mapW +dist,markNum);
}


double *ParticleFilter::Gaussion(void)
{
    int m = 0;
    double var = 400;
    double u = sqrt(var);
    double x[101];
    static double gaga[101];
    for(int i=0;i<=100;i++)
    {
        x[i] = -50+i;
        gaga[i] = 1/(sqrt(2*3.14159)*u)*exp(-(x[i]-m)*(x[i]-m)/(2*u))/0.01994712244439;          //-------------for var=400     <---Warning: QColor::setRgb: RGB parameters out of range
        if(gaga[i] < 0.001)
            gaga[i] = 0.001;
    }
    return gaga;
}

void ParticleFilter::build_LikelihoodMap()
{
    int tIndex;
    double *markNum_;
    markNum_=Gaussion();
    char* imageName = "sks_map";
    image = imread( "/home/kym/kym/src/vision/particle_filter/test.png", 1 );
    cv::Mat gray_img;
    cv::Mat binary_img;
    //      image =    cv::Mat(mapW,mapH,CV_8UC3,Scalar(255,255,255));
    likelihood_map=new cv::Mat(cv::Size(mapW,mapH),CV_8UC1,Scalar(0));
    cv::cvtColor(image,gray_img,CV_RGB2GRAY);
    cv::threshold(gray_img,binary_img,180,255,CV_THRESH_BINARY);

    //mapW = image.cols;
    //mapH = image.rows;
    map = new bool[mapW*mapH];
    likeliHood_map = new double[mapW*mapH];

    for(int i = 0 ;i < mapW*mapH;i++)   likeliHood_map[i] = 0.001;

    for(int i=0;i<binary_img.rows;i++)
    {
        for(int j=0;j<binary_img.cols;j++)
        {
            if(binary_img.data[(i*binary_img.cols)+j]==0)
            {
                tIndex = i*likelihood_map->cols+j;
                map[tIndex] = true;

                for(int m = 0;m<50;m++)
                {
                    likeliHood_surroundMark(tIndex,m,markNum_[50+m]);
                }
            }
        }
    }

    //----------------------------for test
    /*for(int i=0;i<550;i++)
    {
        for(int j=0;j<750;j++)
        {
            std::cout << likeliHood_map[i*750+j] << ",";
        }
    }*/

    //imshow( imageName, *likelihood_map );
    //imshow(imageName,image);
    imwrite("/home/kym/kym/src/vision/particle_filter/likelihood_img.jpg",*likelihood_map);
    waitKey(60);
}

double ParticleFilter::randomX() // return the value between 0 ~ N-1
{
    double x = rand() % (int)mapW;
    while(x<=50 || x>=100)
    {
        x = rand() % (int)mapW;
    }
    return x;
}

double ParticleFilter::randomY() // return the value between 0 ~ N-1
{
    double y = rand() % (int)mapH;
    while(y<=250 || y>=300)
    {
        y = rand() % (int)mapH;
    }
    return y;
}

void ParticleFilter::initParticle_Filter(/*int P_Num,int L_Num*/)
{
//    pNum = P_Num;
//    sensorLineNum = L_Num;
    sensorWall_Dist =  new int[sensorLineNum];
    pAry.clear();

    //mapH=600;
    //mapW=600;

    //gen pNum particle
    for(int i= 0;i < pNum;i++)
    {
        tmp_p.pos(0) = randomX();
        tmp_p.pos(1) = randomY();

        if(trust_imu)
            tmp_yaw = 0;
        else
            tmp_yaw = rand()%360;
        //ROS_INFO("%d:done",i);

        tmp_p.yaw = tmp_yaw*(2*M_PI)/360;
        pAry.push_back(tmp_p);
    }
}
int ParticleFilter::rand_Range()
{
    int randNum = rand()%100;
    return randNum;
}

void ParticleFilter::moveParticle(geometry_msgs::Twist tmp)
{
    int tmp0_noise[12];
    int tmp1_noise[12];
    Vector2d move;
    double rot;
    double move_dir;
    double move_dir_tmp = atan2(tmp.linear.y,tmp.linear.x);
    double move_tmp = sqrt((tmp.linear.x*tmp.linear.x)+(tmp.linear.y*tmp.linear.y));

    move(0) = tmp.linear.x;
    move(1) = tmp.linear.y;
    rot = tmp.angular.z;

    for(int i= 0;i < pAry.size();i++){
        //-------------高斯亂數
        Vector2d G_noise;
        G_noise(0) =0;
        G_noise(1) =0;
        for(int m=0;m<12;m++)
        {
            tmp0_noise[m] = rand_Range();
            tmp1_noise[m] = rand_Range();
        }
        for(int k=0;k<12;k++)
        {
            G_noise(0) +=tmp0_noise[k];
            G_noise(1) +=tmp1_noise[k];
        }

        G_noise(0) = (G_noise(0)/2-300)/300*10; //產生-5~5之間的亂數
        G_noise(1) = (G_noise(1)/2-300)/300*0.52; //產生-0.087~0.087的亂數(-5度~5度)

        move_dir = move_dir_tmp + pAry[i].yaw;

        move(0) = (move_tmp+G_noise(0))*cos(move_dir+G_noise(1));
        move(1) = (move_tmp+G_noise(0))*sin(move_dir+G_noise(1));

        pAry[i].pos = pAry[i].pos + move ;
        pAry[i].yaw = pAry[i].yaw + rot + G_noise(1);

        //re position of particle
        if(pAry[i].pos(0) <0 || pAry[i].pos(0) >=mapW || pAry[i].pos(1) <0 || pAry[i].pos(1) >=mapH)
        {
            pAry[i].pos(0) = randomX();
            pAry[i].pos(1) = randomY();
            if(trust_imu)
                tmp_yaw = 0;
            else
                tmp_yaw = rand()%360;
            pAry[i].yaw = tmp_yaw*(2*M_PI)/360;
        }
    }
}

void ParticleFilter::Sim_SensorModel(Vector3d robot)
{
    double nowDeg = 0;
    double eachDeg = (360.0/sensorLineNum);
    double nowRad;

    sensorWall_Pos.clear();
    for(int i = 0;i < sensorLineNum;i++){
        nowDeg = i * eachDeg-90;    //minus 90 degree for transform of quadrant
        nowRad = robot(2)+nowDeg*2*M_PI/360;

        for(int j=1;;j++)
        {
            Vector2d distPos(j,0);
            Rotation2Dd rot(nowRad);
            Vector2d addPos = rot * distPos;
            Vector2i addPosi = Vector2i(addPos(0),addPos(1));
            Vector2i newPos;
            newPos(0) =  robot(0) + addPosi(0);
            newPos(1) =  robot(1) + addPosi(1);
            int index = newPos(1)*mapW + newPos(0);

            if(map[index]){
                sensorWall_Pos.push_back(newPos);
                sensorWall_Dist[i]=j;
                break;
            }
        }
    }
}

void ParticleFilter::rateGrade()
{
    double rate2;
    for(int i= 0;i < pAry.size();i++){
        double sumGrade = 0;
        double tmp_grade=1;                                 //----for plus
        Vector2i grid_pos(pAry[i].pos(0),pAry[i].pos(1));
        double P_yaw = pAry[i].yaw;

        tpos_wall.clear();
        for(int j=0;j<sensorLineNum;j++)
        {
            double eachDeg = 360/sensorLineNum;
            double nowDeg = j*eachDeg;
            double to_rad = nowDeg*2*3.1415/360-1.57;
            double nowRad = P_yaw + to_rad;
            if(nowRad >= 2*3.14159)
            {
                nowRad = nowRad - 2*3.14159;
            }
            Vector2d addPos;
            addPos(0) = sensorWall_Dist[j]*cos(nowRad);
            addPos(1) = sensorWall_Dist[j]*sin(nowRad);
            Vector2i addPosi = Vector2i(addPos(0),addPos(1));

            tPos = grid_pos + addPosi;

            if(i==(pAry.size()-1))
            {
                tpos_wall.push_back(tPos);
            }

            if(sensorWall_Dist[j]>240)
            {
                rate2 = 0.8;
            }else{
                rate2 = 1;
            }

            if(tPos(0) >= 0 && tPos(0) < mapW && tPos(1) >= 0 && tPos(1) < mapH)
            {
//                double aaaa=likeliHoodMap[tPos(1)*mapW_grid + tPos(0)];             //---------------for Debug
                //std::cout << "likelihood_grade:" << likeliHood_map[tPos(1)*mapW + tPos(0)] << std::endl;
                sumGrade = tmp_grade * likeliHood_map[tPos(1)*mapW + tPos(0)];
                tmp_grade = sumGrade;
//                sumGrade += likeliHoodMap[tPos(1)*mapW_grid + tPos(0)]*rate2;
            }else{
                sumGrade = tmp_grade*0.00001;
                tmp_grade = sumGrade;
            }
        }
        //pAry[i].sumGrade = sumGrade*rate1;
        pAry[i].sumGrade = sumGrade;
        //std::cout << "sumGrade:" << sumGrade << std::endl;
    }
}
void ParticleFilter::rateGrade(Vector3d robot,int sensorWall_Dist[])
{
    double rate2;
    Vector2i robot_pos(robot(0),robot(1));
    sensorWall_Pos.clear();
    for(int i = 0;i<sensorLineNum;i++)
    {
        double eachDeg = 360/sensorLineNum;
        double nowDeg = i*eachDeg;
        double to_rad = nowDeg*2*3.1415/360;
        double nowRad = robot(2) + to_rad;
        if(nowRad >= 2*3.14159)
        {
            nowRad = nowRad - 2*3.14159;
        }
        Vector2d addPos;
        addPos(0) = sensorWall_Dist[i]*cos(nowRad);
        addPos(1) = sensorWall_Dist[i]*sin(nowRad);
        Vector2i addPosi = Vector2i(addPos(0),addPos(1));

        tPos = robot_pos + addPosi;

        sensorWall_Pos.push_back(tPos);

    }
    for(int i= 0;i < pAry.size();i++){
        double sumGrade = 0;
        double tmp_grade=1;                                 //----for plus
        Vector2i grid_pos(pAry[i].pos(0),pAry[i].pos(1));
        double P_yaw = pAry[i].yaw;

        tpos_wall.clear();
        for(int j=0;j<sensorLineNum;j++)
        {
            double eachDeg = 360/sensorLineNum;
            double nowDeg = j*eachDeg;
            double to_rad = nowDeg*2*3.1415/360;
            double nowRad = P_yaw + to_rad;
            if(nowRad >= 2*3.14159)
            {
                nowRad = nowRad - 2*3.14159;
            }
            Vector2d addPos;
            addPos(0) = sensorWall_Dist[j]*cos(nowRad);
            addPos(1) = sensorWall_Dist[j]*sin(nowRad);
            Vector2i addPosi = Vector2i(addPos(0),addPos(1));

            tPos = grid_pos + addPosi;

            if(i==(pAry.size()-1))
            {
                tpos_wall.push_back(tPos);
            }

            if(sensorWall_Dist[j]>240)
            {
                rate2 = 0.5;
            }else{
                rate2 = 1;
            }

            if(tPos(0) >= 0 && tPos(0) < mapW && tPos(1) >= 0 && tPos(1) < mapH)
            {
//                double aaaa=likeliHoodMap[tPos(1)*mapW_grid + tPos(0)];             //---------------for Debug
                //std::cout << "likelihood_grade:" << likeliHood_map[tPos(1)*mapW + tPos(0)] << std::endl;
                sumGrade = tmp_grade * likeliHood_map[tPos(1)*mapW + tPos(0)];
                tmp_grade = sumGrade;
//                sumGrade += likeliHoodMap[tPos(1)*mapW_grid + tPos(0)]*rate2;
            }else{
                sumGrade = tmp_grade*0.00001;
                tmp_grade = sumGrade;
            }
        }
        //pAry[i].sumGrade = sumGrade*rate1;
        pAry[i].sumGrade = sumGrade;
        //std::cout << "sumGrade:" << sumGrade << std::endl;
    }
}
bool ParticleFilter::ISConvergence()
{
    for(int i=1;i<=pAry.size()*0.4;i++)
    {
        pre_pos(0) = pAry[i-1].pos(0);
        pre_pos(1) = pAry[i-1].pos(1);
        pos(0) = pAry[i].pos(0);
        pos(1) = pAry[i].pos(1);
        tmp_sub = pos-pre_pos;
        p_dist = hypot(tmp_sub(0),tmp_sub(1));
        if(p_dist<=5)
            return true;
        else
            return false;
    }
}

void ParticleFilter::roulette_wheel_selection()
{
    allPGardeSum = 0;
    tmp_weight = 0;
    xSum = 0;
    ySum = 0;
    yaw =0;
    sel=0;
    std::vector<Particle,  Eigen::aligned_allocator<Particle> > newAry;

    for(int i= 0;i < pAry.size();i++)
    {
        allPGardeSum+= pAry[i].sumGrade;
    }
    //std::cout << allPGardeSum << std::endl;

    for(int i=0;i<pAry.size();i++)
    {
        tmp_weight += pAry[i].sumGrade/allPGardeSum;
        pAry[i].weight = tmp_weight;
    }

    //std::cout << "pAry[i].weight:" << pAry[0].weight <<std::endl;

    for(int i=0;i<pAry.size();i++)
    {
        tmp_sel = rand()%100;
        sel = tmp_sel/100;

        for(int j=0;j<pAry.size();j++)
        {
            if(sel < pAry[j].weight)
            {
                newAry.push_back(pAry[j]);
                break;
            }
        }
    }
    pAry = newAry;

    for(int m = 0;m<pAry.size();m++)
    {
        xSum += pAry[m].pos(0);
        ySum += pAry[m].pos(1);
        yaw += pAry[m].yaw;
    }

    predictionPos(0) = xSum/pAry.size();
    predictionPos(1) = ySum/pAry.size();
    predictionPos(2) = (yaw)/pAry.size();
}

bool sort_particle(Particle a, Particle b){
    return a.sumGrade > b.sumGrade;
}

void ParticleFilter::tournament_selection()
{
    allPGardeSum = 0;
    tmp_weight = 0;
    xSum = 0;
    ySum = 0;
    yaw =0;
    sel=0;

    std::vector<Particle,  Eigen::aligned_allocator<Particle> > newAry;

    //-------排序
    sort(pAry.begin(),pAry.end(), sort_particle);
    //-------排序

    //-------競爭取前40%
    for(int i=0;i<pAry.size();i++)
    {
        sel = rand()%25;
        newAry.push_back(pAry[sel]);
    }
    pAry = newAry;

    for(int i= 0;i < pAry.size();i++)
    {
        allPGardeSum+= pAry[i].sumGrade;
    }

    for(int m = 0;m<pAry.size();m++)
    {
        tmp_weight = pAry[m].sumGrade/allPGardeSum;
        xSum += pAry[m].pos(0)*tmp_weight;
        ySum += pAry[m].pos(1)*tmp_weight;
        yaw += pAry[m].yaw*tmp_weight;
    }

    predictionPos(0) = xSum;
    predictionPos(1) = ySum;
    predictionPos(2) = yaw;
}

Vector3d ParticleFilter::get_Robot_pos()
{
    Vector3d tmp_robot;
    return tmp_robot;
}

geometry_msgs::Twist ParticleFilter::get_Estimate_pos()
{
    geometry_msgs::Twist estimate_pos;
    estimate_pos.linear.x = predictionPos(0);
    estimate_pos.linear.y = predictionPos(1);
    estimate_pos.angular.z = predictionPos(2);
    return estimate_pos;
}

std::vector<Vector2i> ParticleFilter::get_SensorWall()
{
    return sensorWall_Pos;
}

std::vector<Vector3d> ParticleFilter::get_Particle()
{
    posAry.clear();
    for(int i=0;i < pAry.size();i++){
        Vector3d grid_pos(pAry[i].pos(0),pAry[i].pos(1),pAry[i].yaw);
        posAry.push_back(grid_pos);
    }
    return  posAry;
}
std::vector<Vector2i> ParticleFilter::get_tpwall()
{
    return tpos_wall;
}
