#include <pcl/registration/transforms.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_hull.h>

#include <pcl/common/pca.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include "parameters.h"


const float PI = 3.14159265359;


template <typename PointT> void
SupportPlane<PointT>::getBasis(std::vector<Eigen::Vector3f>& basis)
{
    // Just performs Gram-Schmidt
    Eigen::Vector3f v1 = getNormal().normalized();
    Eigen::Vector3f v2 = Eigen::Vector3f::Random();
    v2 = (v2 - v1.dot(v2) * v1).normalized();
    Eigen::Vector3f v3 = v1.cross(v2);

    //Eigen::Matrix3f fullBasis;
    //fullBasis.col(0) = v1;
    //fullBasis.col(1) = v2;
    //fullBasis.col(2) = v3;

    //std::cout << "full basis: " << std::endl << fullBasis << std::endl;

    basis.clear();
    basis.push_back(v2);
    basis.push_back(v3);
}


template <typename PointT> Eigen::Matrix3f
SupportPlane<PointT>::getCloudPrincipalComponents()
{
  pcl::PCA<PointT> pca;
  pca.setInputCloud(input_cloud_);
  return pca.getEigenVectors();
}


template <typename PointT> float
SupportPlane<PointT>::checkNormalPCA(const Eigen::Vector3f& nvec)
{
  Eigen::Matrix3f pc = getCloudPrincipalComponents();
  Eigen::Vector3f zvec = pc.col(2);
  return abs(zvec.dot(nvec));
}


template <typename PointT> Eigen::Affine3f
SupportPlane<PointT>::getPlaneTransform()
{
  return getPlaneTransform(coefficients_);
}

template <typename PointT> Eigen::Affine3f
SupportPlane<PointT>::getPlaneTransform(pcl::ModelCoefficients::Ptr coefficients)
{
  float a = coefficients->values[0];
  float b = coefficients->values[1];
  float c = coefficients->values[2];
  float d = coefficients->values[3];

  Eigen::Translation3f pos(-a*d, -b*d, -c*d);
  Eigen::Vector3f z(a, b, c);
  float up_dir = 1.0;
  if (z.dot(Eigen::Vector3f(0.0, 0.0, up_dir)) > 0)
  {
    z = -1.0 * z;
  }

  Eigen::Vector3f x(1.0, 0.0, 0.0);
  if (fabs(z.dot(x)) > 1.0 - 1.0e-4) x = Eigen::Vector3f(0.0, 1.0, 0.0);
  Eigen::Vector3f y = z.cross(x).normalized();
  x = y.cross(z).normalized();
  Eigen::Matrix3f rot;

  rot << x[0], x[1], x[2],
         y[0], y[1], y[2],
         z[0], z[1], z[2];
  rot.transposeInPlace();

  Eigen::Affine3f transform = (pos * rot).inverse();
  return transform;
}


template <typename PointT> Eigen::Affine3f
SupportPlane<PointT>::getCameraTransform()
{
    Eigen::Matrix3f rot;
    rot << 0, 0, 1,
           -1, 0, 0,
           0, -1, 0;

    Eigen::Affine3f ret;
    ret = rot;
    return ret.inverse();
}


template <typename PointT> Eigen::Vector3f
SupportPlane<PointT>::getMean()
{
  Eigen::Vector3f mean(0, 0, 0);
  for (size_t i = 0; i < inliers_->indices.size(); ++i) {
    PointT inlier = input_cloud_->points[inliers_->indices[i]];
    mean += Eigen::Vector3f(inlier.x, inlier.y, inlier.z);
  }
  mean /= inliers_->indices.size();

  return mean;
}


template <typename PointT> Eigen::Vector3f
SupportPlane<PointT>::getMean(PointCloudPtr transformedCloudPtr)
{
  assert (transformedCloudPtr->size() == input_cloud_->size());

  Eigen::Vector3f mean(0, 0, 0);
  for (size_t i = 0; i < inliers_->indices.size(); ++i) {
    PointT inlier = transformedCloudPtr->points[inliers_->indices[i]];
    mean += Eigen::Vector3f(inlier.x, inlier.y, inlier.z);
  }
  mean /= inliers_->indices.size();

  return mean;
}


template <typename PointT> Eigen::Vector3f
SupportPlane<PointT>::getCentroid()
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*input_cloud_, inliers_->indices, centroid);
  return Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);
}


template <typename PointT> Eigen::Vector3f
SupportPlane<PointT>::getHullCentroid()
{
  Eigen::Vector4f centroid;
  assert (hull_cloud_->size() > 0);
  pcl::compute3DCentroid(*hull_cloud_, centroid);
  return Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);
}


template <typename PointT> float
SupportPlane<PointT>::degreesBetweenNormals(const Eigen::Vector3f& normal1, const Eigen::Vector3f& normal2)
{
  float angle_between = std::acos(normal1.dot(normal2) / (normal1.norm() * normal2.norm()));
  // Since normal might point the other way...
  float rad = std::min(angle_between, (float) PI - angle_between);
  return rad * 180 / PI;
}


template <typename PointT> int
SupportPlane<PointT>::fitModel(float distance_threshold, bool highest_plane)
{
  std::vector<pcl::ModelCoefficients::Ptr> coeffs_vec;
  std::vector<pcl::PointIndices::Ptr> inliers_vec;

  detectPlanes(input_cloud_, coeffs_vec, inliers_vec, distance_threshold);
  int num_planes = coeffs_vec.size();

  if (num_planes == 0)
  {
    PCL_ERROR("Could not detect any planes in scene\n");
    return (-1);
  }

  std::cout << "Detected " << num_planes << " planes in scene" << std::endl;

  if (!highest_plane || num_planes == 1)  // Just go with first plane detected
  {
    std::cout << "Using 1st plane" << std::endl;
    coefficients_ = coeffs_vec[0];
    inliers_ = inliers_vec[0];
    normal_[0] = coefficients_->values[0];
    normal_[1] = coefficients_->values[1];
    normal_[2] = coefficients_->values[2];
    return (0);
  }

  int highest_ind = 0;
  float max_height = planeHeight(input_cloud_, coeffs_vec[0], inliers_vec[0]);
  std::vector<float> g = params().g;
  Eigen::Vector3f gvec(g[0], g[1], g[2]);
  std::cout << "Plane height 0: " << max_height << std::endl;
  std::cout << "Num inliers 0: " << inliers_vec[0]->indices.size() << std::endl;
  std::cout << "Deg 0: " << degreesBetweenNormals(gvec,
      Eigen::Vector3f(coeffs_vec[0]->values[0], coeffs_vec[0]->values[1],
      coeffs_vec[0]->values[2])) << std::endl;
  for (int i=1; i < num_planes; i++)
  {
    // TODO Check filtering inliers doesn't mess up indexing
    float h = planeHeight(input_cloud_, coeffs_vec[i], inliers_vec[i]);
    std::cout << "Plane height " << i << ": " << h << std::endl;
    std::cout << "Num inliers " << i << ": " << inliers_vec[i]->indices.size() << std::endl;
    std::cout << "Deg " << i << ": " << degreesBetweenNormals(gvec,
      Eigen::Vector3f(coeffs_vec[i]->values[0], coeffs_vec[i]->values[1],
      coeffs_vec[i]->values[2])) << std::endl;
    if (h > max_height)
    {
      highest_ind = i;
      max_height = h;
    }
  }

  std::cout << "Picked plane: " << highest_ind << std::endl;
  coefficients_ = coeffs_vec[highest_ind];
  inliers_ = inliers_vec[highest_ind];

  normal_[0] = coefficients_->values[0];
  normal_[1] = coefficients_->values[1];
  normal_[2] = coefficients_->values[2];

  std::cout << normal_ << std::endl;

  return (0);
}


template <typename PointT> float
SupportPlane<PointT>::planeHeight(PointCloudPtr cloud, pcl::ModelCoefficients::Ptr coefficients,
  pcl::PointIndices::Ptr inliers)
{
  float height = 0;
  PointCloudPtr transformed_cloud(new PointCloud);
  pcl::transformPointCloud(*cloud, *transformed_cloud, (Eigen::Affine3f) getPlaneTransform(coefficients).linear());

  int num_inliers = inliers->indices.size();
  for (int i=0; i < num_inliers; i++)
  {
    height += transformed_cloud->at(inliers->indices[i]).z;
  }
  height /= num_inliers;
  return height;
}


template <typename PointT> void
SupportPlane<PointT>::segmentPlane(PointCloudPtr cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, float distance_threshold)
{
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(params().plane_ransac_max_iters);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
}


template <typename PointT> int
SupportPlane<PointT>::detectPlanes(PointCloudPtr input_cloud,
                                   std::vector<pcl::ModelCoefficients::Ptr>& coeffs_vec,
                                   std::vector<pcl::PointIndices::Ptr>& inliers_vec,
                                   float distance_threshold)
{
  float NORMAL_EPS_ANGLE = params().normal_eps_angle;
  std::cout << "NORMAL_EPS_ANGLE: " << NORMAL_EPS_ANGLE << std::endl;
  int MIN_PLANE_CLOUD_SIZE = params().min_plane_cloud_size;

  // TODO Fall back to PCA normal if gravity vector not provided
  std::vector<float> g = params().g;
  Eigen::Vector3f gvec(g[0], g[1], g[2]);
  gvec.normalize();
  std::cout << "gravity vector: " << gvec << std::endl;

  PointCloudPtr full_cloud = input_cloud_->makeShared();
  while (full_cloud->size() > MIN_PLANE_CLOUD_SIZE)
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    Eigen::Vector3f normal(0, 0, 0);

    std::cout << "Finding a plane" << std::endl;

    inliers->indices.clear();
    segmentPlane(full_cloud->makeShared(), coeffs, inliers, distance_threshold);

    if (inliers->indices.size() < MIN_PLANE_CLOUD_SIZE) {
      break;
    }

    normal[0] = coeffs->values[0];
    normal[1] = coeffs->values[1];
    normal[2] = coeffs->values[2];
    normal.normalize();

    if (inliers->indices.size() > MIN_PLANE_CLOUD_SIZE &&
        degreesBetweenNormals(normal, gvec) < NORMAL_EPS_ANGLE)
    {
      std::cout << "Passed angle test" << std::endl;
      if (checkSupportPlane(full_cloud->makeShared(), coeffs, inliers))
      {
        std::cout << "Passed checkSupportPlane" << std::endl;
        inliers_vec.push_back(inliers);
        coeffs_vec.push_back(coeffs);
      }
    }

    filterInliers(full_cloud, full_cloud, inliers);
  }

  if (coeffs_vec.size()  == 0)
  {
    PCL_ERROR("Could not estimate a planar model for input cloud\n");
    return (-1);
  }

  return (0);
}


template <typename PointT> bool
SupportPlane<PointT>::checkSupportPlane(PointCloudPtr cloud, pcl::ModelCoefficients::Ptr coefficients,
  pcl::PointIndices::Ptr inliers)
{
  /* FIXME computeHull broken
  PointCloudPtr hull_cloud(new PointCloud);
  std::cout << "cloud size: " << cloud->size() << std::endl;
  computeHull(cloud, hull_cloud, coefficients, inliers);

  PointCloudPtr points_above_hull(new PointCloud);
  pointsAboveHull(cloud, points_above_hull, hull_cloud, coefficients);

  if (points_above_hull->size() < params().min_pts_above_hull)
  {
    return false;
  }
  */

  return true;
}


template <typename PointT> void
SupportPlane<PointT>::computeHull()
{
  computeHull(input_cloud_, hull_cloud_, coefficients_, inliers_);
}


template <typename PointT> void
SupportPlane<PointT>::computeHull(PointCloudPtr cloud, PointCloudPtr hull_cloud,
    pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers)
{
  // TODO Replace w/ performReconstruction2D?

  PointCloudPtr cloud_proj(new PointCloud);

  // First project inliers onto plane

  pcl::ProjectInliers<PointT> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud);
  pcl::IndicesPtr inds(new std::vector<int>(inliers->indices));
  proj.setIndices(inds);
  proj.setModelCoefficients(coefficients);
  proj.filter(*cloud_proj);

  /*
  pcl::visualization::PCLVisualizer viz("SupportPlane Viz");
  viz.addCoordinateSystem(0.1);
  viz.addPointCloud(cloud_proj);
  // TODO Visualize normal/axes
  while (!viz.wasStopped())
  {
    viz.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  */

  // Compute convex hull of projected inliers

  hull_.setDimension(2);
  hull_.setInputCloud(cloud_proj);
  hull_.reconstruct(*hull_cloud);
  std::cout << "Convex hull has " << hull_cloud->size() << " data points"
      << std::endl;

}


template <typename PointT> void
SupportPlane<PointT>::filterInliers(PointCloudPtr filtered_cloud)
{
  filterInliers(input_cloud_, filtered_cloud, inliers_);
}


template <typename PointT> void
SupportPlane<PointT>::filterInliers(PointCloudPtr cloud,
                                    PointCloudPtr filtered_cloud)
{
  filterInliers(cloud, filtered_cloud, inliers_);
}


template <typename PointT> void
SupportPlane<PointT>::filterInliers(PointCloudPtr cloud,
                                    PointCloudPtr filtered_cloud,
                                    pcl::PointIndices::Ptr inliers)
{
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setKeepOrganized(true);
  extract.setNegative(true);
  extract.filter(*filtered_cloud);
}


template <typename PointT> void
SupportPlane<PointT>::pointsAbovePlane(PointCloudPtr cloud,
                               PointCloudPtr filtered_cloud,
                               pcl::ModelCoefficients::Ptr coefficients)
{
  PointCloudPtr transformed_cloud(new PointCloud);

  Eigen::Affine3f transform = getPlaneTransform(coefficients);
  pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
  filter_field(transformed_cloud, "z",
      params().plane_z_thresh.at(0),
      params().plane_z_thresh.at(1));
  pcl::transformPointCloud(*transformed_cloud, *filtered_cloud, transform.inverse());
}


template <typename PointT> void
SupportPlane<PointT>::pointsAbovePlane(PointCloudPtr cloud,
                               PointCloudPtr filtered_cloud)
{
  pointsAbovePlane(cloud, filtered_cloud, coefficients_);
}


template <typename PointT> void
SupportPlane<PointT>::pointsAboveHull(PointCloudPtr cloud,
                              PointCloudPtr filtered_cloud,
                              PointCloudPtr hull_cloud,
                              pcl::ModelCoefficients::Ptr coefficients)
{
  PointCloudPtr cloud_above_plane(new PointCloud);
  pointsAbovePlane(cloud, cloud_above_plane, coefficients);

  PointCloudPtr cloud_above_hull(new PointCloud);
  BOOST_FOREACH(PointT p, cloud_above_plane->points)
  {
    if (pcl::isPointIn2DPolygon(p, *hull_cloud))
      cloud_above_hull->points.push_back(p);
  }

  filtered_cloud->points = cloud_above_hull->points;
}


template <typename PointT> void
SupportPlane<PointT>::pointsAboveHull(PointCloudPtr cloud,
                              PointCloudPtr filtered_cloud)
{
  pointsAboveHull(cloud, filtered_cloud, hull_cloud_, coefficients_);
}


template <typename PointT> void
SupportPlane<PointT>::transformCloud(PointCloudPtr cloud,
                             PointCloudPtr transformed_cloud)
{
  pcl::transformPointCloud(*cloud, *transformed_cloud, getPlaneTransform());
}


template <typename PointT> void
SupportPlane<PointT>::untransformCloud(PointCloudPtr transformed_cloud,
               PointCloudPtr cloud)
{
  pcl::transformPointCloud(*transformed_cloud, *cloud, getPlaneTransform().inverse());
}


template <typename PointT> void
SupportPlane<PointT>::visualize()
{
  visualize(input_cloud_, hull_cloud_, normal_);
}


template <typename PointT> void
SupportPlane<PointT>::visualize(PointCloudPtr cloud, PointCloudPtr hull_cloud,
    Eigen::Vector3f normal)
{
    pcl::visualization::PCLVisualizer viz("SupportPlane Viz");
    viz.addCoordinateSystem(0.1);
    viz.addPointCloud(cloud);
    //viz.addPolygon<PointT>(hull_cloud, 0.0, 1.0, 0.0);
    // TODO Visualize normal/axes
    while (!viz.wasStopped())
    {
      viz.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}


template <typename PointT> void
SupportPlane<PointT>::projectPointOnPlane(PointT& pt, PointT& proj_pt)
{
    Eigen::Vector3f p(pt.x, pt.y, pt.z);
    Eigen::Vector3f n = getNormal().normalized();
    std::vector<float> c = coefficients_->values;
    // ax + by + cz + d = 0
    Eigen::Vector3f p_plane(p[0], p[1], (-c[0]*p[0] - c[1]*p[1] - c[3]) / c[2]);

    Eigen::Vector3f p_proj = p - (p - p_plane).dot(n) * n;
    proj_pt.x = p_proj(0);
    proj_pt.y = p_proj(1);
    proj_pt.z = p_proj(2);
}
