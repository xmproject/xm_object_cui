#ifndef FILTER
#define FILTER

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
//#include "detect.h"

//using namespace pcl;
typedef pcl::PointXYZRGBA PointT;

class Filter
{
public:
Filter();
~Filter();
    struct Box2DPoint
    {
    float x_left_down;
    float y_left_down;
    float x_right_up;
    float y_right_up;
    };

    pcl::PointCloud<PointT>::Ptr  RemovePlane (const pcl::PointCloud<PointT>::Ptr input_cloud);
    std::vector<Filter::Box2DPoint>  Cluster(const pcl::PointCloud<PointT>::Ptr input_cloud);
};


#endif // FILTER_H_INCLUDED
