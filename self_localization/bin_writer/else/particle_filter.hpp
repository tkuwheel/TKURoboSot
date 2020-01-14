/*
 *   particle_filter.hpp
 *
 *   Date:2015
 *
 *   Author: Chien-Ming Lin
 *
 */
#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include "ros/ros.h"
#include <string.h>
#include "iostream"
#include "std_msgs/Header.h"
#include "sensor_msgs/LaserScan.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/Twist.h"
//#include "particle_filter/encoder.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace Eigen;
using namespace cv;
//using namespace std;

struct PStruct{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector2d pos;
    double sumGrade;
    double yaw;
    double weight;
};

typedef struct PStruct Particle;


class ParticleFilter
{

private:
    Mat image;
    bool *map;
    Mat *likelihood_map;        // for Painting Map
    double *likeliHood_map;     // for RateGrade
    int mapH;
    int mapW;
    int pNum;
    int sensorLineNum;
    int *sensorWall_Dist;

    //for tournament_selection or roulette_wheel_selection
    double tmp_yaw;
    double allPGardeSum;
    double tmp_weight;
    double xSum;
    double ySum;
    double yaw;
    double tmp_sel;
    int sel;


    //ISConvergence()
    Vector2d pre_pos;
    Vector2d pos;
    Vector2d tmp_sub;
    double p_dist;

    Vector2i tPos;
    Vector3d Robot;
    Vector3d predictionPos;
    std::vector<Vector3d> posAry;
    std::vector<Vector2i> tpos_wall;
    std::vector<Vector2i> sensorWall_Pos;
    std::vector<Particle,  Eigen::aligned_allocator<Particle> > pAry;

public:
    ParticleFilter(int p_Num,int sensor_LineNum,int map_H,int map_W);
    ~ParticleFilter(){};
    /************************************************************/
    /************************* Read Map *************************/
    /************************************************************/
    void build_LikelihoodMap();
    double *Gaussion();
    void likeliHood_markWant(int wantIndex,double markNum);
    void likeliHood_surroundMark(int Index,int dist,double markNum);
    /***********************************************************/
    /******************** Monte Carlo Method *******************/
    /***********************************************************/
    double randomX();
    double randomY();
    void initParticle_Filter(/*int P_Num,int L_Num*/);
    int rand_Range();
    void moveParticle(geometry_msgs::Twist tmp);
    void Sim_SensorModel(Vector3d robot);
    void rateGrade();
    void rateGrade(Vector3d robot,int sensorWall_Dist[]);
    bool ISConvergence();
    void roulette_wheel_selection();
    void tournament_selection();
    /************************************************************/
    /*********************** Retrun value ***********************/
    /************************************************************/
    Vector3d get_Robot_pos();
    geometry_msgs::Twist get_Estimate_pos();
    std::vector<Vector2i> get_SensorWall();
    std::vector<Vector3d> get_Particle();
    std::vector<Vector2i> get_tpwall();
};

#endif
