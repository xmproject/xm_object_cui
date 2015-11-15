
#include "tracking.h"
#include <opencv2/opencv.hpp>
#include <tld_utils.h>
#include <iostream>
#include <sstream>
#include <TLD.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>


using namespace cv;
using namespace std;

Tracking::Tracking()
{
}



void Tracking::Tracking1(const cv::Rect& obj,Mat  curr_image)
{
    box.x=obj.x;
    box.y=obj.y;
    box.width=obj.width;
    box.height=obj.height;
    Mat frame;
    curr_image.copyTo(frame);
    cvtColor(frame, last_gray, CV_RGB2GRAY);
    status=true;
    tld.read(1);
    bb_file = fopen("bounding_boxes.txt","w");
    tld.init(last_gray,box,bb_file);
    tl=true;
}

bool  Tracking::trackobj(Mat curr_image)
{

    Mat frame;
    //ros::Publisher givedata=n.advertise<follower::num>("tld_people_position_estimation",1000);
    //follower::num data;
    //bb_file = fopen("bounding_boxes.txt","w");
    //int frames = 1;
    //int detections = 1;
        //get frame
    curr_image.copyTo(frame);
    cvtColor(frame, current_gray, CV_RGB2GRAY);
    tld.processFrame(last_gray,current_gray,pts1,pts2,pbox,status,tl,bb_file);
    if (status)
    {
            drawPoints(frame,pts1);
            drawPoints(frame,pts2,Scalar(0,255,0));
            drawBox(frame,pbox);

           /* cout<<"computing ave_dep"<<endl;
            float ave_dep = average_depth(ic.curr_image_depth, pbox);
            cout<<"average depth = "<<ave_dep/1000.0<<endl;*/
	    /*fx = 525.0  # focal length x
	    fy = 525.0  # focal length y
	    cx = 319.5  # optical center x
	    cy = 239.5  # optical center y*/
	   /* float ux=(pbox.x+pbox.x+pbox.width)/2.0;
	    float uy=(pbox.y+pbox.y+pbox.height)/2.0;
	    float worldx=(ux-319.5)*(ave_dep)/525.0;
	    float worldy=(uy-239.5)*(ave_dep)/525.0;
	    data.a=1;
	    data.y=-worldx/1000.0;
	    data.z=-worldy/1000.0;
	    data.x=ave_dep/1000.0;
	    if(data.x>=0)
		{cout<<data.a<<" "<<data.x<<" "<<data.y<<" "<<data.z<<endl;
	    givedata.publish(data);}*/


        }
        //Display
        imshow("TLD", frame);
        waitKey(3);
        //swap points and images
        swap(last_gray,current_gray);
        pts1.clear();
        pts2.clear();

        if(status)
            return true;
        else
            return false;

    //fclose(bb_file);
}
