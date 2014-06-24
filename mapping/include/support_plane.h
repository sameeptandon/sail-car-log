#pragma once

#include <iostream>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

// TODO If have normals may just want to use pcl::OrganizedMultiPlaneSegmentation?

template <typename PointT>
class SupportPlane
{

public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  SupportPlane() : coefficients_(new pcl::ModelCoefficients),
                   inliers_(new pcl::PointIndices),
                   hull_cloud_(new PointCloud),
                   normal_(0.0, 0.0, 0.0)
                   {}

  // Set input cloud of full scene
  inline void
  setInputCloud(PointCloudPtr cloud)
  {
    input_cloud_ = cloud;
  }

  // Get normal vector to the plane
  inline Eigen::Vector3f
  getNormal()
  {
      return normal_;
  }

  // Return model coefficients
  inline pcl::ModelCoefficientsPtr
  getCoeffs()
  {
      return coefficients_;
  }

  // Get basis for the plane (2 vectors perpendicular to normal and each other)
  void getBasis(std::vector<Eigen::Vector3f>& basis);

  // Get basis estimated using PCA of input cloud
  Eigen::Matrix3f getCloudPrincipalComponents();

  // Check normal vector by checking that dot product with normal
  // computed using PCA exceeds threshold
  // Not needed if gravity vector is known
  float checkNormalPCA(const Eigen::Vector3f& nvec);

  // Get Eigen transform so support plane normal points up
  Eigen::Affine3f getPlaneTransform();
  Eigen::Affine3f getPlaneTransform(pcl::ModelCoefficients::Ptr coefficients);

  // Get transform so +z in direction camera pointing towards after
  // applying plane transform
  Eigen::Affine3f getCameraTransform();

  // Get center of inliers
  Eigen::Vector3f getMean();

  // Get center of inliers of transformed cloud
  Eigen::Vector3f getMean(PointCloudPtr transformedCloudPtr);

  // Get centroid of inliers
  Eigen::Vector3f getCentroid();

  // Get centroid of convex hull of inliers
  Eigen::Vector3f getHullCentroid();

  // Check normals close
  float degreesBetweenNormals(const Eigen::Vector3f& normal1, const Eigen::Vector3f& normal2);

  /*
   * Fit plane model parameters and initialize normal;
   * If highest_plane is true, sets plane model to highest plane found using
   * detectPlanes
   */
  int fitModel(float distance_threshold, bool highest_plane=true);

  /*
   * Find all planes in a scene;
   * First finds primary plane and uses it to check normals of remaining planes;
   * Checks that initial normal agrees with PCA-estimated normal of input_cloud_ or approximate gravity vector if provided;
   * Also checks that remaining planes have points (possible objects) above their convex hulls
   */
  int detectPlanes(PointCloudPtr cloud, std::vector<pcl::ModelCoefficients::Ptr>& coefficients,
      std::vector<pcl::PointIndices::Ptr>& inliers, float distance_threshold);

  // Checks that a plane's convex hull has points above it
  bool checkSupportPlane(PointCloudPtr cloud, pcl::ModelCoefficients::Ptr coefficients,
      pcl::PointIndices::Ptr inliers);

  // Find the average z-value of plane inliers
  float planeHeight(PointCloudPtr cloud, pcl::ModelCoefficients::Ptr coefficients,
      pcl::PointIndices::Ptr inliers);

  // RANSAC plane estimation
  void segmentPlane(PointCloudPtr cloud, pcl::ModelCoefficients::Ptr coefficients,
      pcl::PointIndices::Ptr inliers, float distance_threshold);

  // Compute convex hull of support plane (to check whether points above)
  void computeHull();
  void computeHull(PointCloudPtr cloud, PointCloudPtr hull_cloud,
      pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers);

  // Input cloud with current inliers removed
  void filterInliers(PointCloudPtr filtered_cloud);
  // Take in input cloud and remove current inliers
  void filterInliers(PointCloudPtr cloud, PointCloudPtr filtered_cloud);
  // Take in input cloud and remove specified inliers
  void filterInliers(PointCloudPtr cloud, PointCloudPtr filtered_cloud,
      pcl::PointIndices::Ptr inliers);

  // Get points above the hull
  void pointsAboveHull(PointCloudPtr cloud, PointCloudPtr filtered_cloud);
  void pointsAboveHull(PointCloudPtr cloud, PointCloudPtr filtered_cloud,
      PointCloudPtr hull_cloud, pcl::ModelCoefficients::Ptr coefficients);

  // Get points above the plane
  void pointsAbovePlane(PointCloudPtr cloud, PointCloudPtr filtered_cloud);
  void pointsAbovePlane(PointCloudPtr cloud, PointCloudPtr filtered_cloud,
      pcl::ModelCoefficients::Ptr coefficients);

  // Transform point cloud so table normal faces up
  void transformCloud(PointCloudPtr cloud, PointCloudPtr transformed_cloud);

  // Transform cloud back
  void untransformCloud(PointCloudPtr transformed_cloud, PointCloudPtr cloud);

  void projectPointOnPlane(PointT& pt, PointT& proj_pt);

  // For debugging
  void visualize();
  void visualize(PointCloudPtr cloud, PointCloudPtr hull, Eigen::Vector3f normal);

  PointCloudPtr hull_cloud_;  // FIXME

protected:

  PointCloudPtr input_cloud_;
  pcl::ModelCoefficients::Ptr coefficients_;
  pcl::PointIndices::Ptr inliers_;
  pcl::ConvexHull<PointT> hull_;
  Eigen::Vector3f normal_;

private:


};

#include "support_plane.hpp"
