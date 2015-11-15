#include "rosgetimage.h"

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/console/print.h>
//OpenCV specific inccludes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//tf
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
//using namespace std;
//using namespace cv;


#define FOCAL 525.0

rosgetimage::rosgetimage(ros::NodeHandle n):
        nh_(n),
        it_(nh_)//,
        //info_msg(new sensor_msgs::CameraInfoConstPtr)
{
    std::cout<<"Init cloudTraver strat"<<std::endl;
    ready=false;
    cloudReady=false;
    imageReady=false;
    image_sub_ = it_.subscribeCamera("/camera/rgb/image_color", 1,
                               &rosgetimage::imageCb, this);
    //if you want a depth image follow this
    //image_sub_depth_ = it_.subscribe("/camera/depth_registered/image_raw",1,
    //                                &ImageConverter::imageCbDepth, this);
    cloud_sub = n.subscribe ("/camera/depth_registered/points", 1, &rosgetimage::cloudCb,this);
    std::cout<<"Init cloudTraver OK"<<std::endl;
}
rosgetimage::~rosgetimage() {}

void rosgetimage::cloudCb (const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  //std::cout<<"cloudCB"<<std::endl;
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::fromROSMsg (*inputCloud, cloud);
  currCloud=cloud.makeShared () ;
  cloudReady=true;
  if(imageReady)
  {
      ready=true;
  }
  else
  {
          //std::cout<<"but image not ready"<<std::endl;
  }
}

void rosgetimage::imageCb(const sensor_msgs::ImageConstPtr& inputImage,
                          const sensor_msgs::CameraInfoConstPtr& inputInfo_msg)
{
    info_msg=inputInfo_msg;
    //std::cout<<"imageCB"<<std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(inputImage, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception & e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    currImage = cv_ptr->image.clone();
    //    cv_ptr->image.copyTo(curr_image);
    imageReady=true;
    if(cloudReady)
    {
      ready=true;
    }
    else
    {
        //std::cout<<"but cloud not ready"<<std::endl;
    }
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rosgetimage::getCloud()
{
    cloudReady=false;
    ready=false;
    return currCloud;
}

cv::Mat rosgetimage::getImage()
{
    imageReady=false;
    ready=false;
    return currImage;
}

bool rosgetimage::isReady()
{
    return ready;
}

/*void cloudTraver::Point3Dto2D(cv::Point3d point3D,cv::Point2d& point2D)
{

     cam_model_.fromCameraInfo(info_msg);
     tf::StampedTransform transform;
     std::cout<<cam_model_.tfFrame()<<std::endl;
     try
     {
       ros::Time acquisition_time = info_msg->header.stamp;
       ros::Duration timeout(1.0 / 30);
       tf_listener_.waitForTransform(cam_model_.tfFrame(), "/camera_rgb_optical_frame",
                                     acquisition_time, timeout);
       tf_listener_.lookupTransform(cam_model_.tfFrame(), "/camera_rgb_optical_frame",
                                    acquisition_time, transform);
     }
     catch (tf::TransformException& ex) {
       ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
       return;
     }

     point2D = cam_model_.project3dToPixel(point3D);
     std::cout<<point3D.x<<"    "<<point3D.y<<"    "<<point3D.z<<std::endl;
     std::cout<<point2D.x<<"    "<<point2D.y<<std::endl;

    if(point3D.z<0.01)
    {
        pcl::console::print_error("the point's 'z' Is too low.(0.0) would return\n");
        point2D.x=0;
        point2D.y=0;
    }
    else if(point3D.z>5.0)
    {
        pcl::console::print_error("the point's 'z' Is too large (0.0) would return\n");
        point2D.x=0;
        point2D.y=0;
    }
    else
    {
        point2D.x=point3D.x*FOCAL/point3D.z+320;
        point2D.y=point3D.y*FOCAL/point3D.z+240;
    }
}*/
