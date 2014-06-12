#pragma once

#include <boost/shared_ptr.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include "point_defs.h"
#include <pcl/correspondence.h>
#include <pcl/PointIndices.h>

void get_box_corners(pcl::PointXYZ min_pt, pcl::PointXYZ max_pt, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > corners);

template <typename PointT>
void load_cloud(std::string pcd_path, boost::shared_ptr<pcl::PointCloud<PointT> > cloud);

template <typename PointT>
void load_clouds(std::vector<std::string> pcd_paths, boost::shared_ptr<pcl::PointCloud<PointT> > cloud);

template<typename PointT>
void align_clouds_viz(const boost::shared_ptr<pcl::PointCloud<PointT> > src_cloud, const boost::shared_ptr<pcl::PointCloud<PointT> > tgt_cloud, boost::shared_ptr<pcl::PointCloud<PointT> > aligned_cloud, const pcl::Correspondences& correspondences, bool viz_normals=false);

template <typename PointT>
void project_cloud(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, cv::Mat& translation_vector, cv::Mat& rotation_vector, cv::Mat& intrinsics, cv::Mat& distortions, std::vector<cv::Point2f>& imagePoints);

// Wrapper around projectCloud which converts rotation matrix
// to rvec and uses Eigen matrices instead of cv::Mat
template <typename PointT>
void project_cloud_eigen(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, const Eigen::Vector3f& translation_vector, const Eigen::Matrix3f& rotation_matrix, const Eigen::Matrix3f& intrinsics, const Eigen::VectorXf distortions, std::vector<cv::Point2f>& imagePoints);
//void project_cloud_eigen(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, const Eigen::Vector3f& translation_vector, const Eigen::Matrix3f& rotation_matrix, const Eigen::Matrix3f& intrinsics, const Eigen::VectorXf distortions, Eigen::MatrixX2f& imagePoints);

template <typename PointT>
void filter_cloud(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, boost::shared_ptr<pcl::PointCloud<PointT> > filtered_cloud, std::vector<int>& filtered_indices, const std::string& filter_dim, float range_min, float range_max);

// Filters lidar cloud to be projected to camera
// Assumes cloud is in the camera frame
template<typename PointT>
void filter_lidar_cloud(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, boost::shared_ptr<pcl::PointCloud<PointT> > filtered_cloud, std::vector<int>& filtered_indices);

template <typename PointT>
void filter_field(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, std::string field, float lower, float upper);

template <typename PointT>
void extract_clusters_euclidean(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, std::vector<pcl::PointIndices>& cluster_indices, float cluster_tol, int min_cluster_size, int max_cluster_size);

template <typename PointT>
void extract_clusters_euclidean(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > >& clusters, float cluster_tol, int min_cluster_size, int max_cluster_size);

#include "cloud_utils.hpp"
