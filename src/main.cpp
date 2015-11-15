//从文件中获取图片名称，载入，分批进行处理，最后识别
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/console/print.h>
//OpenCV specific includes
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>

#include <vector>
#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include "rosgetimage.h"
#include "Filter.h"
#include "detect.h"
#include  <follower/pubdata.h>
#include <tracking.h>
//using namespace std;
//using namespace cv;

cv::Rect box;
std::string  object_name;
int flag=0;
int sum1;
int num;
int zhao=0;
string name[31];
//std::stringstream ss;
enum Process_State {free1,recognition};
Process_State State;

std::vector<cv::Mat>  descriptor[31];
std::vector<cv::Mat> image_obj[31];
pcl::visualization::CloudViewer viewer("X");
//Tracking gen;


std::vector<Filter::Box2DPoint> my_points;

int main(int argc, char** argv)
{
     ros::init(argc, argv, "object");
     ros::NodeHandle n;
     ros::ServiceClient client;
     ros::Rate waiting_rate(30);
     cv::initModule_nonfree();

     rosgetimage ct(n);
     Detection detect;
     while(!ct.isReady())
     {
         ros::spinOnce();
         waiting_rate.sleep();
     }
    State=free1;
    cvNamedWindow("CurrentImage",CV_WINDOW_AUTOSIZE);
     cv::Mat image;
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudP;
    int  s;
    while(ros::ok())
    {
        while(!ct.isReady())
        {
          ros::spinOnce();
        }

        image=ct.getImage();
        cloudP=ct.getCloud();

        if(cloudP->empty())
        {
            std::cout<<"No pointCloud passed into this process, fuck you no points you play MAOXIAN!"<<std::endl;
            continue;
        }
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_RGBA(new pcl::PointCloud<pcl::PointXYZRGBA>);
        *cloud_RGBA=*cloudP;
        Filter filter;

        //Detection detect;
        my_points=filter.Cluster(cloud_RGBA);
        cloud_RGBA=filter.RemovePlane(cloud_RGBA);
        if(!my_points.empty())
        {
            switch(State)
            {
                case free1:
                    break;


                case recognition:
                {
                    //if(zhao==0)
                    {
                        detect.detect_object(image,my_points,num,descriptor[num]);
                    //detect.tracking();
                        //gen.Tracking1(detect.track_object,image);
                    cv::rectangle(image,cv::Point(detect.track_object.x,detect.track_object.y),cv::Point(detect.track_object.x+detect.track_object.width,detect.track_object.y+detect.track_object.height),cv::Scalar(255,0,0));
                        /*std_msgs::String massg;
                        std::stringstream ss11;
                        ss11<<detect.track_object.x<<","<<detect.track_object.y<<","<<detect.track_object.width<<","<<detect.track_object.height;
                        massg.data=ss11.str();
                        while(1)
                        {
                            client = n.serviceClient<follower::pubdata>("givedata");
                            follower::pubdata srv;
                            srv.request.a=massg.data;
                            if (client.call(srv))
                            {
                                zhao=1;
                                break;
                            }
                        }*/


                    }

                        //gen.trackobj(image);

                     break;
                }
            }
        }

        viewer.showCloud(cloud_RGBA);
        cv::imshow("CurrentImage", image);
        char temp=cvWaitKey(33);
        switch(temp)
        {
            case 'q':
                return 0;
            case 'f':
                State=free1;
                break;
           case 'r':
                if(flag==0)
                {
                    cv::FileStorage fs("/home/cuiyufang/object_recognition/object.xml", cv::FileStorage::READ);
                    sum1=(int)fs["sum"];
                    int i;
                    cv::FileNode name_obj = fs["name"];
                    cv::FileNodeIterator  it_obj = name_obj.begin(), it_obj_end = name_obj.end();
                    for(i=1;it_obj!=it_obj_end;++it_obj,++i)
                        {name[i]=(string)(*it_obj);

                        }
                    //std::cout<<name[1]<<" "<<name[2]<<std::endl;

                    for(i=1;i<=sum1;i++)
                    {
                        //ss<<i;
                        //descriptor[i]=(std::vector<cv::Mat>)(fs[name[i]]["Descriptor"]);
                        cv::FileNode descriptor1 = fs[name[i]]["Descriptor"];
                        cv::FileNodeIterator  it = descriptor1.begin(), it_end = descriptor1.end();

                       for(;it!=it_end;++it)
                        {
                            // std::vector<cv::Mat> aaa;
                             it>>descriptor[i];
                        }


                        //ss.clear();
                    }
                   // cout<<descriptor[1].size()<<" "<<descriptor[2].size()<<endl;
                    for(i=1;i<=sum1;i++)
                    {
                        //ss<<i;
                        //descriptor[i]=(std::vector<cv::Mat>)(fs[name[i]]["Descriptor"]);
                        cv::FileNode image1 = fs[name[i]]["Image"];
                        cv::FileNodeIterator  it_image = image1.begin(), it_image_end = image1.end();

                       for(;it_image!=it_image_end;++it_image)
                        {
                            // std::vector<cv::Mat> aaa;
                             it_image>>image_obj[i];
                        }


                        //ss.clear();
                    }
                    //cout<<image_obj[1].size()<<" "<<image_obj[2].size()<<endl;
                    flag=1;
                }
                cout<<"please input the Command"<<endl;
                cin>>s;
               // if(s==1000)
                    //num=1000;
                if(s>100)
                    num=s%100;
                else
                    num=s%10;
                State=recognition;
                zhao=0;
                break;

        }
    }
}

















