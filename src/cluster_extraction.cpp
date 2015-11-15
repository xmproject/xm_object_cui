#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include<pcl/filters/passthrough.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
// PCL specific includes

//OpenCV specific includes
#include <string>
#include <fstream>

#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/console/print.h>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <ros/ros.h>
#include "rosgetimage.h"

#define FOCAL 525.0

typedef pcl::PointXYZRGBA PointT;

struct Box2DPoint
{
  float x_left_down;
  float y_left_down;
  float x_right_up;
  float y_right_up;
};

int main (int argc, char** argv)
{
  // Read in the cloud data
  //pcl::PCDReader reader;
  ros::init (argc, argv, "test");
  ros::NodeHandle n;
  ros::Rate waiting_rate(30);
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>), cloud_f (new pcl::PointCloud<PointT>);
  rosgetimage ct(n);
  while(!ct.isReady())
    {
         ros::spinOnce();
         waiting_rate.sleep();
     }
while(ros::ok())
{
  //reader.read ("inputCloud0.pcd", *cloud);
   while(!ct.isReady())
    {
        ros::spinOnce();
    }
    cloud=ct.getCloud();
     if(cloud->empty())
        {
            std::cout<<"No pointCloud passed into this process"<<std::endl;
            continue;
        }

  //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;

  //passthrough
  pcl::PointCloud<PointT>::Ptr cloud_passthrough(new pcl::PointCloud<PointT>);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0,2.0);
  pass.filter(*cloud);


  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PointT> vg;
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  //std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);


  pcl::visualization::PCLVisualizer viewer_f ("plane_removed");
  viewer_f.addPointCloud(cloud_filtered, "cloud_filtered");


  pcl::visualization::PCLVisualizer viewer ("cluster");
  std::vector<Box2DPoint> my_points;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    std::string name = ss.str();
    viewer.addPointCloud (cloud_cluster, name);
    writer.write<PointT> (name, *cloud_cluster, false); //*
    std::cout << "loaded" << std::endl;

    std::cout << cloud_cluster->size() << std::endl;

    float x_min=999,y_min=999,z_min=999;
    float x_max=0,y_max=0,z_max=0;
    for(int i=0;i<cloud_cluster->size();i++)
    {
        //std::cout << cloud_cluster->points[i].x << "   " << cloud_cluster->points[i].y <<"    " << cloud_cluster->points[i].z << std::endl;
        if(cloud_cluster->points[i].x<x_min)
            x_min=cloud_cluster->points[i].x;
        if(cloud_cluster->points[i].y<y_min)
            y_min=cloud_cluster->points[i].y;
        if(cloud_cluster->points[i].z<z_min)
            z_min=cloud_cluster->points[i].z;

        if(cloud_cluster->points[i].x>x_max)
            x_max=cloud_cluster->points[i].x;
        if(cloud_cluster->points[i].y>y_max)
            y_max=cloud_cluster->points[i].y;
        if(cloud_cluster->points[i].z>z_max)
            z_max=cloud_cluster->points[i].z;
    }

    //viewer.addCube(x_min, x_max, y_min, y_max, z_min, z_max, 0, 255, 0, name);

    Box2DPoint pp;
    pp.x_left_down = x_min*FOCAL/z_min+320;
    pp.y_left_down = y_min*FOCAL/z_min+240;
    pp.x_right_up = x_max*FOCAL/z_max+320;
    pp.y_right_up = y_max*FOCAL/z_max+240;
    my_points.push_back(pp);
    j++;
  }

}
  return (0);
}
