#include <cstdio>
#include <math.h>
#include <omp.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <std_msgs/Int32MultiArray.h>

#define TEST "/home/kym/kym/src/vision/particle_filter/ground.png"

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

using namespace std;
using namespace cv;
typedef unsigned char BYTE;

int main(int argc,char **argv){
    string vision_path = ros::package::getPath("self_localization");
	string FILE_PATH = "/bin_writer/table.bin";
    string Filename = vision_path + FILE_PATH;
    const char *Filename_Path = Filename.c_str();
    if(ifstream(Filename)){
    //==========================================================
        Mat self_map(FIELD_HEIGHT+50*2, FIELD_WIDTH+50*2, CV_8UC3, Scalar(0,0,0));
        line(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE1), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE1), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE6), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE6), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE1), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE6), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE1), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE6), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE4,self_map.rows/2+YLINE1), Point(self_map.cols/2+XLINE4,self_map.rows/2+YLINE6), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE2,self_map.rows/2+YLINE3), Point(self_map.cols/2+XLINE2,self_map.rows/2+YLINE4), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE6,self_map.rows/2+YLINE3), Point(self_map.cols/2+XLINE6,self_map.rows/2+YLINE4), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE3,self_map.rows/2+YLINE2), Point(self_map.cols/2+XLINE3,self_map.rows/2+YLINE5), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2+YLINE2), Point(self_map.cols/2+XLINE5,self_map.rows/2+YLINE5), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE2,self_map.rows/2+YLINE3), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE3), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE2,self_map.rows/2+YLINE4), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE4), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE3,self_map.rows/2+YLINE2), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE2), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE3,self_map.rows/2+YLINE5), Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE5), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE6,self_map.rows/2+YLINE3), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE3), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE6,self_map.rows/2+YLINE4), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE4), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2+YLINE2), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE2), Scalar(255,255,255), 1);
        line(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2+YLINE5), Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE5), Scalar(255,255,255), 1);
        circle(self_map, Point(self_map.cols/2,self_map.rows/2), CENTER_RADIUS, Scalar(255, 255, 255), 1);
        ellipse(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE1), Size(CENTER_RADIUS2,CENTER_RADIUS2), 0, 180, 270, Scalar(255,255,255), 1);
        ellipse(self_map, Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE1), Size(CENTER_RADIUS2,CENTER_RADIUS2), 0, 270, 360, Scalar(255,255,255), 1);
        ellipse(self_map, Point(self_map.cols/2+XLINE1,self_map.rows/2+YLINE6), Size(CENTER_RADIUS2,CENTER_RADIUS2), 0, 90, 180, Scalar(255,255,255), 1);
        ellipse(self_map, Point(self_map.cols/2+XLINE7,self_map.rows/2+YLINE6), Size(CENTER_RADIUS2,CENTER_RADIUS2), 0, 0, 90, Scalar(255,255,255), 1);

        ellipse(self_map, Point(self_map.cols/2+XLINE3,self_map.rows/2), Size(CENTER_RADIUS,CENTER_RADIUS), 0, 90, 270, Scalar(255,255,255), 1);
        ellipse(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2), Size(CENTER_RADIUS,CENTER_RADIUS), 0, 0, 90, Scalar(255,255,255), 1);
        ellipse(self_map, Point(self_map.cols/2+XLINE5,self_map.rows/2), Size(CENTER_RADIUS,CENTER_RADIUS), 0, 270, 360, Scalar(255,255,255), 1);

        circle(self_map, Point(self_map.cols/2+XLINE8,self_map.rows/2+YLINE2), 1, Scalar(255, 255, 255), -1);
        circle(self_map, Point(self_map.cols/2+XLINE8,self_map.rows/2+YLINE5), 1, Scalar(255, 255, 255), -1);
        circle(self_map, Point(self_map.cols/2+XLINE9,self_map.rows/2+YLINE2), 1, Scalar(255, 255, 255), -1);
        circle(self_map, Point(self_map.cols/2+XLINE9,self_map.rows/2+YLINE5), 1, Scalar(255, 255, 255), -1);

        Mat ground = self_map.clone();
        Mat dis(ground.rows, ground.cols, CV_8UC3, Scalar(0,0,0));

        std::vector<int> xline;
        std::vector<int> yline;
        int start_x = -XLINE1-50;;
        int start_y = -YLINE1-50;
        int end_x = XLINE1+50;;
        int end_y = YLINE1+50;
        int x_length = end_x-start_x+1;
        int y_length = end_y-start_y+1;
        double *distance_matrix;//=new double[x_length*y_length];
        distance_matrix = (double*) malloc(sizeof(double)*x_length*y_length);
        for(int i=0; i<dis.rows; i++){
            for(int j=0; j<dis.cols; j++){
                distance_matrix[i * x_length + j]=255;
            }
        }
        //distance_matrix[0] = 1.2222;
        //imshow("self_map",self_map);

        double max=0,min=9999;

    //====================================
        cout<<"start"<<endl;
        
        #pragma omp parallel for
        for(int i=0;i<dis.rows;i++){//y
            #pragma omp parallel for
            for(int j=0;j<dis.cols;j++){//x
                //===================================
                #pragma omp parallel for
                for(int k=-255;k<255;k++){//y
                    #pragma omp parallel for
                    for(int l=-255;l<255;l++){//x
                        if((i+k)<0||(i+k)>=dis.rows-1)continue;
                        if((j+l)<0||(j+l)>=dis.cols-1)continue;
                        if(ground.data[((i+k) * ground.cols + (j+l)) * 3 + 0] == 255){
                            double distance = sqrt(k*k+l*l);
                            if(distance>255)distance =255;
                            int color = 255-distance-50;
                            int ori_color = dis.data[(i * dis.cols + j) * 3 + 0];
                            if(ori_color<color){
                                dis.data[(i * dis.cols + j) * 3 + 0]=color;
                                dis.data[(i * dis.cols + j) * 3 + 1]=color;
                                dis.data[(i * dis.cols + j) * 3 + 2]=color;
                                distance_matrix[i*x_length+j]=distance;
                            }
                        }
                    }
                }
                //====================================
            }
        }
        for(int i=0;i<x_length*y_length;i++)
        {
            if(distance_matrix[i]>max)max=distance_matrix[i];
            if(distance_matrix[i]<min)min=distance_matrix[i];
        }
        cout<<"max:"<<max<<",min:"<<min<<endl;
        cout<<"finish"<<endl;
        imshow("dis",dis);

        string name = ros::package::getPath("self_localization") + "/bin_writer/distance_matrix2.png";
        imwrite(name,dis);
    //===================================
        FILE *file = fopen(Filename_Path, "rb+"); //開啟檔案來寫
        fwrite( distance_matrix, sizeof(double), x_length*y_length , file );
        fclose(file);
        waitKey(5000);
    }else{
        cout<<"can not find the bin file.\n";
    }    
    return 0;
}

