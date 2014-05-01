#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointXYZ> CloudXYZ;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> NormalCloud;
typedef pcl::PointXYZINormal PointTNormal;
typedef pcl::PointCloud<PointTNormal> PointCloudWithNormals;
