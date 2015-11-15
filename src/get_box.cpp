#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#define FOCAL 525.0

typedef pcl::PointXYZRGBA PointT;

struct Box2DPoint
{
  float x_left_down;
  float y_left_down;
  float x_right_up;
  float y_right_up;
};


pcl::PointCloud<PointT>::Ptr RemovePlane (const pcl::PointCloud<PointT>::Ptr input_cloud)
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
  
  pcl::PointCloud<PointT>::Ptr cloud_colorthrough (new pcl::PointCloud<PointT>);
  int r,g,b;
  for (size_t i = 0; i < cloud_filter->points.size (); i++)
  {	
    r = cloud_filter->points[i].r;
    g = cloud_filter->points[i].g;
    b = cloud_filter->points[i].b;
		
    if (r > 200 && r < 250)
    if (g > 200 && g < 250)
    if (b > 200 && b < 250)continue;
		
    cloud_colorthrough->points.push_back (cloud_filter->points[i]);
  }
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;
   
  return cloud_colorthrough;
}

std::vector<Box2DPoint> Cluster(const pcl::PointCloud<PointT>::Ptr input_cloud)
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

        float x_min=999,y_min=999,z_min=999;
        float x_max=0,y_max=0,z_max=0;    
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

        Box2DPoint pp;
        pp.x_left_down = x_min*FOCAL/z_min+320;
        pp.y_left_down = y_min*FOCAL/z_min+240;
        pp.x_right_up = x_max*FOCAL/z_max+320;
        pp.y_right_up = y_max*FOCAL/z_max+240;  
        my_points.push_back(pp);
        j++;
   }

    return my_points;
}

int main(int argc, char** argv)
{
    pcl::PCDReader reader;
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    reader.read ("inputCloud0.pcd", *cloud);
    
    std::vector<Box2DPoint> my_points;
    my_points = Cluster (cloud);
    for (std::vector<Box2DPoint>::const_iterator it = my_points.begin (); it != my_points.end (); ++it)
    {
        std::cout << it->x_left_down << " " << it->y_left_down << std::endl;
        std::cout << it->x_right_up << " " << it->y_right_up << "\n" << std::endl;
    }  
    return 0;
}


