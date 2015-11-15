#ifndef UTI
#define UTI

#include <opencv2/opencv.hpp>
#include <tld_utils.h>
#include <iostream>
#include <sstream>
#include <TLD.h>
#include <stdio.h>
#include <LKTracker.h>
#include <FerNNClassifier.h>
#include <std_msgs/String.h>



class Tracking
{
public:
    Tracking();
    void Tracking1(const cv::Rect& obj,cv::Mat  curr_image);
    cv::Rect box;
    cv::Mat  last_gray;
    cv::Mat current_gray;
    cv::vector<cv::Point2f> pts1;
    cv::vector<cv::Point2f> pts2;
    bool status;
    bool trackobj( cv::Mat curr_image);

    FILE *bb_file;
    TLD tld;
     BoundingBox pbox;
     bool tl;
private:


};

#endif
