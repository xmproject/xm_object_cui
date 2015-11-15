#ifndef TRACKING_H_INCLUDED
#define TRACKING_H_INCLUDED

#include <opencv2/opencv.hpp>
#include <tld_utils.h>
#include <iostream>
#include <sstream>
#include <TLD.h>
#include <stdio.h>

#include <std_msgs/String.h>



class Tracking
{
public:

    Tracking();
    void Tracking1(const cv::Rect& obj,cv::Mat  curr_image);
    cv::Rect box;
    ros::Publisher givedata;
    cv::Mat  last_gray;
    cv::Mat current_gray;
    cv::vector<cv::Point2f> pts1;
    cv::vector<cv::Point2f> pts2;
    bool status;
    void trackobj( cv::Mat curr_image);
    TLD tld;
     FILE *bb_file;
     BoundingBox pbox;
     bool tl;

};


#endif // TRACKING_H_INCLUDED
