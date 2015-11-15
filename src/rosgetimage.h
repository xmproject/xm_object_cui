/*write bt AirSmith
 * 2015 5 26
 * a class to make a brige between ROSmsg and PCL&OpenCV
 * never forget to run rqt and let the OpennNI driver "registed"
 */

#ifndef ROSGETIMAGE
#define ROSGETIMAGE
#include <vector>
#include <iostream>
//ros specific includes
#include <ros/ros.h>
//PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
//opencv specific includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//tf
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"

//using namespace std;
//using namespace cv;

class rosgetimage
{
public:
  rosgetimage(ros::NodeHandle n);
  ~rosgetimage();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud();
  cv::Mat getImage();
  bool isReady();

 // void Point3Dto2D(cv::Point3d point3D,cv::Point2d& point2D);
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& inputCloud);
  void imageCb(const sensor_msgs::ImageConstPtr& inputImage,
const sensor_msgs::CameraInfoConstPtr& info_msg);
  ros::NodeHandle nh_;
private:
  //tf::TransformListener tf_listener_;
  //image_geometry::PinholeCameraModel cam_model_;

  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  //image_transport::Subscriber image_sub_depth_;

  cv::Mat currImage;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currCloud;

  bool ready;
  bool cloudReady;
  bool imageReady;

  sensor_msgs::CameraInfoConstPtr info_msg;
  //image_geometry::PinholeCameraModel cam_model_;
  ros::Subscriber cloud_sub;
};
#endif
