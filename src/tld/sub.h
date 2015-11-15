#ifndef SUB
#define SUB

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "follower/pubdata.h"

class sub
{
public:
     sub();
     ~sub();
     ros::NodeHandle main;
     ros::Subscriber subdata;
     
     cv::Rect box;
     bool file;
     bool chatterCallback(follower::pubdata::Request  &req,
             follower::pubdata::Response &res);
};

#endif
