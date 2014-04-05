#include "point_defs.h"
#include <pcl/correspondence.h>


template<typename PointT>
void align_clouds_viz(const boost::shared_ptr<pcl::PointCloud<PointT> > src_cloud, const boost::shared_ptr<pcl::PointCloud<PointT> > tgt_cloud, boost::shared_ptr<pcl::PointCloud<PointT> > aligned_cloud, const pcl::Correspondences& correspondences, bool viz_normals=false);

#include "cloud_viz.hpp"
