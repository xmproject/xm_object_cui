#include "sub.h"
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace std;
using namespace cv;
sub::sub()
{
     
     //subdata = main.subscribe("givedata",1,&sub::chatterCallback, this);
     file=false;
}
sub::~sub()
{
}
bool sub::chatterCallback(follower::pubdata::Request  &req,
             follower::pubdata::Response &res)
{
  ROS_INFO("I heard: [%s]", req.a.c_str());
  string line=req.a;
  istringstream linestream(line);
  string x1,y1,x2,y2;
  getline (linestream,x1, ',');
  getline (linestream,y1, ',');
  getline (linestream,x2, ',');
  getline (linestream,y2, ',');
  int x = atoi(x1.c_str());// = (int)file["bb_x"];
  int y = atoi(y1.c_str());// = (int)file["bb_y"];
  int w = atoi(x2.c_str());// = (int)file["bb_w"];
  int h = atoi(y2.c_str());// = (int)file["bb_h"];
  box = Rect(x,y,w,h);
  
  file=true;
  res.b=1;
  return true;
	
}
    
