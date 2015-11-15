//从文件中获取图片名称，载入，分批进行处理，最后识别
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/ros.h>
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

#include "rosgetimage.h"
#include "Filter.h"
#include "Train.h"
//#include "detect.h"

//using namespace std;
//using namespace cv;

cv::Rect box;
std::string  object_name;


enum Process_State {save, free1,quit,train1};
Process_State State;

pcl::visualization::CloudViewer viewer("X");



/*typedef struct Box2D
    {
    float x_left_down;
    float y_left_down;
    float x_right_up;
    float y_right_up;
    }Box2DPoint;*/
std::vector<Filter::Box2DPoint> my_points;

int main(int argc, char** argv)
{
     ros::init(argc, argv, "xm_object_recognition");
     ros::NodeHandle n;
     ros::Rate waiting_rate(30);
     cv::initModule_nonfree();

     rosgetimage ct(n);
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
    int k;
    int sum;
     Train trainmodel;
    while(ros::ok())
    {
        while(!ct.isReady())
        {
          ros::spinOnce();
        }

        image=ct.getImage();
        cloudP=ct.getCloud();
        //cout<<"1"<<endl;
        if(cloudP->empty())
        {
            std::cout<<"No pointCloud passed into this process, fuck you no points you play MAOXIAN!"<<std::endl;
            continue;
        }
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_RGBA(new pcl::PointCloud<pcl::PointXYZRGBA>);
        *cloud_RGBA=*cloudP;
        Filter filter;
        //cout<<"1"<<endl;

        //Detection detect;
        my_points=filter.Cluster(cloud_RGBA);
        cloud_RGBA=filter.RemovePlane(cloud_RGBA);
        //cout<<1<<endl;
        if(!my_points.empty())
        {
            //cout<<1<<endl;
            switch(State)
            {
                case free1:
                cout<<my_points[0].x_left_down<<" "<<my_points[0].y_left_down<<" "<<my_points[0].x_right_up<<" "<<my_points[0].y_right_up<<endl;
                for(k=0;k<my_points.size();k++)
                {
                 if((my_points[k].x_left_down<0)||(my_points[k].y_left_down<0)||(my_points[k].x_right_up>image.cols)||(my_points[k].y_right_up>image.rows))
                {
                    k++;
                    continue;
                }
                if(((my_points[k].x_right_up-my_points[k].x_left_down)>300)||((my_points[k].y_right_up-my_points[k].y_left_down)>350))
                {
                    k++;
                    continue;
                }
		 if(((my_points[k].x_right_up-my_points[k].x_left_down)<0)||((my_points[k].y_right_up-my_points[k].y_left_down)<0))
        	{
            	k++;
            	continue;
        	}
                 cv::rectangle(image,cv::Point(my_points[k].x_left_down,my_points[k].y_left_down),cv::Point(my_points[k].x_right_up,my_points[k].y_right_up),cv::Scalar(255,0,0));
                }
                    break;

                case save:
                {
                    cv::Mat temp=image.clone();
                    cv::rectangle(image,cv::Point(my_points[0].x_left_down,my_points[0].y_left_down),cv::Point(my_points[0].x_right_up,my_points[0].y_right_up),cv::Scalar(255,0,0));
                    cv::imshow("CurrentImage", image);
                    if (cvWaitKey(33) == 'q')
                        return 0;
                    if(object_name!="n")
                    {
                    box.x=my_points[0].x_left_down;
                    box.y=my_points[0].y_left_down;
                    box.width=my_points[0].x_right_up-my_points[0].x_left_down;
                    box.height=my_points[0].y_right_up-my_points[0].y_left_down;
                    //*** This path specify location where the file of object's informations, 
                    // you need replace ros workspace to what you are using ***
                    cv::imwrite("/home/ubuntu/Desktop/ros_ws/xm_test_ws/src/xm_object_cui/config/object_recognition"+object_name+".jpg",temp(box));
                    char key = cvWaitKey(10);
                    if ( key == 27)
                        {break;}
                    //*** This path specify location where the file of object's informations, 
                    // you need replace ros workspace to what you are using ***
                    std::ofstream of("/home/ubuntu/Desktop/ros_ws/xm_test_ws/src/xm_object_cui/config/object_recognition/name.txt",ios::app);
                    of<<object_name<<endl;
                    of.close();
                    }
                    State=free1;
                    break;
                }
                case train1:
                {

                    if(trainmodel.train(sum))
                        std::cout<<"train down"<<std::endl;
                    else
                         std::cout<<"train fail"<<std::endl;
                    State=free1;
                    break;
                }
                /*case recognition:
                 {
                    detect.detect_object(trainmodel,image,my_points,s,sum);
                    //detect.tracking();
                    cv::rectangle(image,cv::Point(detect.track_object.x,detect.track_object.y),cv::Point(detect.track_object.x+detect.track_object.width,detect.track_object.y+detect.track_object.height),cv::Scalar(255,0,0));

                    break;
                }*/



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
            case 's':
                State=save;
                std::cout<<"please input the object name"<<std::endl;
                std::cin>>object_name;
                break;
            /*case 'r':
                State=recognition;
                cout<<"please input the Command"<<endl;
                cin>>s;

                break;*/
            case 't':
                std::cout<<"please input the totle number of objects"<<std::endl;
                std::cin>>sum;
                State=train1;
                break;
        }
    }
}

















