#ifndef DETECT_H_INCLUDED
#define DETECT_H_INCLUDED


#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include <vector>
#include <cstdio>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/contrib/contrib.hpp>
//#include "train.h"
#include "Filter.h"
#include <ros/ros.h>


using namespace std;
//using namespace cv;

class Detection
{
public:
    struct Point
    {
        int x;
        int y;
    };
struct Box2DPoint
    {
    float x_left_down;
    float y_left_down;
    float x_right_up;
    float y_right_up;
    };
    cv::Rect track_object;
    //void tracking();
    //ros::NodeHandle m;
  struct compare
{
bool operator ()(Point a,Point b)
{
        return a.y>b.y;
}
};
    //Detection();
   bool detect_object(cv::Mat image,const std::vector<Filter::Box2DPoint>& my_points,const int& s,const vector<cv::Mat>&  des,const vector<cv::MatND>& hist);




};


#endif // DETECT_H_INCLUDED
