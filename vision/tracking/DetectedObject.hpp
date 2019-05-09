#include <iostream> 
#include <string> 
using namespace std;

#ifndef DETECTEDOBJECT_HPP
#define DETECTEDOBJECT_HPP

struct DetectedObject
{
    DetectedObject();
    int dis_max;     //pix
    int dis_min;     //pix
    int ang_max;     //pix
    int ang_min;     //pix
    int x;           //pix
    int y;           //pix
    int angle;       //pix
    double distance; //pix
    int size;
    string LR;
};

DetectedObject::DetectedObject():
    dis_max(0),
    dis_min(0),
    ang_max(0),
    ang_min(0),
    x(0),
    y(0),
    angle(0),
    distance(0.0),
    size(0),
    LR("Null")
{
}
#endif
