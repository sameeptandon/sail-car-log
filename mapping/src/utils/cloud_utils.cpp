#include "utils/cloud_utils.h"


void get_box_corners(pcl::PointXYZ min_pt, pcl::PointXYZ max_pt, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > corners)
{
  corners->clear();
  corners->push_back(pcl::PointXYZ(min_pt.x, min_pt.y, max_pt.z));
  corners->push_back(pcl::PointXYZ(min_pt.x, max_pt.y, max_pt.z));
  corners->push_back(pcl::PointXYZ(max_pt.x, max_pt.y, max_pt.z));
  corners->push_back(pcl::PointXYZ(max_pt.x, min_pt.y, max_pt.z));
  corners->push_back(pcl::PointXYZ(min_pt.x, min_pt.y, min_pt.z));
  corners->push_back(pcl::PointXYZ(min_pt.x, max_pt.y, min_pt.z));
  corners->push_back(pcl::PointXYZ(max_pt.x, max_pt.y, min_pt.z));
  corners->push_back(pcl::PointXYZ(max_pt.x, min_pt.y, min_pt.z));
}
