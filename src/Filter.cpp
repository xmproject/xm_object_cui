#include "Filter.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include<pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/common/common.h>
#include<pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#define FOCAL 525.0

//typedef pcl::PointCloudRGBA PointT;
//using namespace pcl;

typedef pcl::PointXYZRGBA PointT;

Filter::Filter()
{

}
Filter::~Filter()
{

}
pcl::PointCloud<PointT>::Ptr Filter::RemovePlane (const pcl::PointCloud<PointT>::Ptr input_cloud)
{
  //Passthrough
  pcl::PointCloud<PointT>::Ptr cloud_passthrough(new pcl::PointCloud<PointT>);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0,1.0);
 // pass.setFilterFieldName("x");
  //pass.setFilterLimits(-0.5,0.5);
  pass.filter(*cloud_passthrough);
    
  //downsample
  pcl::PointCloud<PointT>::Ptr cloud_downsample (new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud (cloud_passthrough);
  vg.setLeafSize (0.01f, 0.01f, 0.01f); //
  vg.filter (*cloud_downsample);
  
  //remove plane
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_removeplane(new pcl::PointCloud<PointT>);
  *cloud_removeplane = *cloud_downsample;
  //pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);   //
  seg.setDistanceThreshold (0.01); //0.02
  int i = 0, nr_points = (int) cloud_removeplane->points.size ();
  while (cloud_removeplane->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_removeplane);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Remove the planar inliers, extract the rest
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_removeplane);
    extract.setIndices(inliers);
    extract.setNegative (true);
    extract.filter (*cloud_removeplane); 
  }
  return cloud_removeplane;
}

std::vector<Filter::Box2DPoint> Filter::Cluster(const pcl::PointCloud<PointT>::Ptr input_cloud)
{
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    cloud_filtered = RemovePlane (input_cloud);    

    //Creating  the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (1000);//
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

   // pcl::visualization::PCLVisualizer viewer ("cluster");
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

      //  viewer.addPointCloud (cloud_cluster, "cloud");

        float x_min=1000000,y_min=1000000,z_min=1000000;
        float x_max=-1000000,y_max=-1000000,z_max=-1000000;
	float zm1,zm2,zm3,zm4;    
       /* for(int i=0;i<cloud_cluster->size();i++)
        {
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
        }*/
	for(int i=0;i<cloud_cluster->size();i++)
        {
            if(cloud_cluster->points[i].x<x_min)
                {x_min=cloud_cluster->points[i].x;zm1=cloud_cluster->points[i].z;}
            if(cloud_cluster->points[i].y<y_min)
                {y_min=cloud_cluster->points[i].y;zm2=cloud_cluster->points[i].z;}
            //if(cloud_cluster->points[i].z<z_min)
              //  z_min=cloud_cluster->points[i].z;

            if(cloud_cluster->points[i].x>x_max)
               { x_max=cloud_cluster->points[i].x;zm3=cloud_cluster->points[i].z;}
            if(cloud_cluster->points[i].y>y_max)
                {y_max=cloud_cluster->points[i].y;zm4=cloud_cluster->points[i].z;}
            //if(cloud_cluster->points[i].z>z_max)
               // z_max=cloud_cluster->points[i].z;    
        }
   // viewer.addCube(x_min, x_max, y_min, y_max, z_min, z_max, 0, 255, 0, "cloud");

        Box2DPoint pp;
        pp.x_left_down = x_min*FOCAL/zm1+320;
        pp.y_left_down = y_min*FOCAL/zm2+240;
        pp.x_right_up = x_max*FOCAL/zm3+320;
        pp.y_right_up = y_max*FOCAL/zm4+240;  
        my_points.push_back(pp);
        j++;
   }

    return my_points;
}


/*pcl::PointCloud<PointT>::Ptr  Filter::RemovePlane (const pcl::PointCloud<PointT>::Ptr input_cloud)
{
  //Passthrough
  pcl::PointCloud<PointT>::Ptr cloud_passthrough(new pcl::PointCloud<PointT>);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0,2.0);
  pass.filter(*cloud_passthrough);

  //downsample
  pcl::PointCloud<PointT>::Ptr cloud_downsample (new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud (cloud_passthrough);
  vg.setLeafSize (0.01f, 0.01f, 0.01f); //
  vg.filter (*cloud_downsample);

  //remove plane
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_removeplane(new pcl::PointCloud<PointT>);
  *cloud_removeplane = * cloud_downsample;
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);   //
  seg.setDistanceThreshold (0.01); //0.02
  int i = 0, nr_points = (int) cloud_removeplane->points.size ();
  while (cloud_removeplane->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_downsample);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Remove the planar inliers, extract the rest
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_removeplane);
    extract.setIndices(inliers);
    extract.setNegative (true);
    extract.filter (*cloud_removeplane);
  }
  return cloud_removeplane;
}

std::vector<Filter::Box2DPoint>   Filter::Cluster(const pcl::PointCloud<PointT>::Ptr input_cloud)
{
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    cloud_filtered = RemovePlane (input_cloud);

    //Creating  the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (1000);//
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    //pcl::visualization::PCLVisualizer viewer ("cluster");
    std::vector<Filter::Box2DPoint> my_points;
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        //cout<<1<<endl;
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //viewer.addPointCloud (cloud_cluster, "cloud");


        float x_min=999,y_min=999,z_min=999;
        float x_max=-999,y_max=-999,z_max=-999;
        for(int i=0;i<cloud_cluster->size();i++)
        {
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

   // viewer.addCube(x_min, x_max, y_min, y_max, z_min, z_max, 0, 255, 0, "cloud");

        Filter::Box2DPoint pp;
        pp.x_left_down = x_min*FOCAL/z_min+320;
        pp.y_left_down = y_min*FOCAL/z_min+240;
        pp.x_right_up = x_max*FOCAL/z_max+320;
        pp.y_right_up = y_max*FOCAL/z_max+240;
        my_points.push_back(pp);
        j++;
   }
    return my_points;
}*/



