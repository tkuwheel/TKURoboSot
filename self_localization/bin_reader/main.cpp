#include <cstdio>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <fstream>
#include <std_msgs/Int32MultiArray.h>

#define DEG2RAD  3.14159/180
#define N_PARTICLE 600
#define TO_RAD M_PI/180.0

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

#define DISTANCE_MATRIX_WIDTH 700
#define DISTANCE_MATRIX_HEIGHT 500

using namespace std;
using namespace cv;
typedef unsigned char BYTE;

std::vector<int> xline;
std::vector<int> yline;
int start_x = -XLINE1-50;
int start_y = -YLINE1-50;
int end_x = XLINE1+50;
int end_y = YLINE1+50;
int x_length = end_x-start_x+1;
int y_length = end_y-start_y+1;
double *distance_matrix;

double distance(double x, double y)
{
  if((abs((int)x)<=end_x) &&
     (abs((int)y)<=end_y))
    return distance_matrix[((int)(y)-start_y)*x_length+(int)(x)-start_x];
  else
  {
    //        std::cout << "[FieldMatrix] index out of bound\n";
    return 500.0;
  }
}
int main(int argc,char **argv){
    distance_matrix = (double*) malloc(sizeof(double)*x_length*y_length);
    //#define TEST ros::package::getPath("vision")+"/bin_reader/errortable.bin"
    string vision_path = ros::package::getPath("self_localization");
    string FILE_PATH = "/bin_reader/table.bin";
    string Filename = vision_path + FILE_PATH;
    const char *Filename_Path = Filename.c_str();
    // open the file:
    streampos fileSize;
    if(ifstream(Filename)){
        std::ifstream file(Filename_Path, ios::binary);
        // get its size:
        file.seekg(0, ios::end);
        fileSize = file.tellg();
        file.seekg(0, ios::beg);
        // read the data:
        vector<BYTE> fileData(fileSize);
        file.read((char *)distance_matrix, fileSize);
        cout<<"read bin finish"<<endl;
        double max=0,min=9999;
        for(int i=0;i<x_length*y_length;i++)
        {
            if(distance_matrix[i]>max)max=distance_matrix[i];
            if(distance_matrix[i]<min)min=distance_matrix[i];
        }
        cout<<"max:"<<max<<",min:"<<min<<endl;

        cv::Mat distance_map(DISTANCE_MATRIX_HEIGHT, DISTANCE_MATRIX_WIDTH, CV_8UC3, Scalar(255,255,255));
        for(int i=0; i<DISTANCE_MATRIX_WIDTH; i++)
        {
            for(int j=0; j<DISTANCE_MATRIX_HEIGHT; j++)
            {
                auto color = (int)(distance((double)i-DISTANCE_MATRIX_WIDTH/2,(double)j-DISTANCE_MATRIX_HEIGHT/2))+50;
                auto px = (color>255)?    (255)    :    (color<0?0:color);
                px = 255-px;
                line(distance_map, Point(i,j), Point(i,j), Scalar(px,px,px), 1);
                //int color = distance_matrix[j*(DISTANCE_MATRIX_WIDTH)+i];
                //color = 255-color;
                //line(distance_map, Point(i,j), Point(i,j), Scalar(color,color,color), 1);
                
            }
        }

        /*for(int i=0; i<distance_map.rows; i++){
            for(int j=0; j<distance_map.cols; j++){
                int color = distance_matrix[i * distance_map.cols + j];
                line(distance_map, Point(j,i), Point(j,i), Scalar(color,color,color), 1);
            }
        }*/

        cv::imshow("distance_map",distance_map);
        cv::waitKey(5000);
    }else{
        cout<<"can not find the bin file.\n";
    }
	return 0;
}


