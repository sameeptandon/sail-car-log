#include <vector>

#include <boost/shared_ptr.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

// NOTE Also removes NaNs
template <typename PointT>
void load_cloud(std::string pcd_path, boost::shared_ptr<pcl::PointCloud<PointT> > cloud)
{
    if (pcl::io::loadPCDFile(pcd_path, *cloud) < 0)
    {
        std::cout << "Error loading input point cloud " << pcd_path << std::endl;
        throw;
    }
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
}

template <typename PointT>
void load_clouds(std::vector<std::string> pcd_paths, boost::shared_ptr<pcl::PointCloud<PointT> > cloud)
{
    BOOST_FOREACH(std::string pcd_path, pcd_paths)
    {
        pcl::PointCloud<PointXYZ>::Ptr new_cloud(new pcl::PointCloud<PointT>());
        load_cloud(pcd_path, new_cloud);
        *cloud += *new_cloud;
    }
}

template<typename PointT>
void align_clouds_viz(const boost::shared_ptr<pcl::PointCloud<PointT> > src_cloud, const boost::shared_ptr<pcl::PointCloud<PointT> > tgt_cloud, boost::shared_ptr<pcl::PointCloud<PointT> > aligned_cloud, const pcl::Correspondences& correspondences, bool viz_normals)
{
    pcl::visualization::PCLVisualizer viz("align clouds viz");
    //viz.addCoordinateSystem(3.0);

    // Add point clouds and normals

    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(tgt_cloud, 0, 255, 0);
    viz.addPointCloud(tgt_cloud, green, "tgt_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(tgt_cloud, 0, 0, 255);
    viz.addPointCloud(src_cloud, blue, "src_cloud");

    if (viz_normals)
    {
        viz.addPointCloudNormals<PointT>(tgt_cloud, 1, 0.5f, "tgt_cloud_normals", 0);
        viz.addPointCloudNormals<PointT>(src_cloud, 1, 0.5f, "src_cloud_normals", 0);
    }

    // Add correspondences

    if (correspondences.size() > 0)
        viz.addCorrespondences<PointT>(src_cloud, tgt_cloud, correspondences, "correspondences", 0);

    // Add the final aligned cloud

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZINormal> red(aligned_cloud, 255, 0, 0);
    viz.addPointCloud(aligned_cloud, red, "aligned_cloud");

    viz.spin();
}


template <typename PointT>
void project_cloud(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, cv::Mat& translation_vector, cv::Mat& rotation_vector, cv::Mat& intrinsics, cv::Mat& distortions, std::vector<cv::Point2f>& imagePoints) {
  int length = cloud->points.size();
  cv::Mat objectPoints(length, 3, CV_32FC1);

  // Copy over PCL cloud points into cv::Mat
  for(int i = 0; i < length; i++) {
    objectPoints.at<float>(i, 0) = cloud->points[i].x;
    objectPoints.at<float>(i, 1) = cloud->points[i].y;
    objectPoints.at<float>(i, 2) = cloud->points[i].z;
  }

  //std::cout << rotation_vector << std::endl;
  //std::cout << translation_vector << std::endl;
  //std::cout << intrinsics << std::endl;
  //std::cout << distortions << std::endl;
  cv::projectPoints(objectPoints, rotation_vector, translation_vector, intrinsics, distortions, imagePoints);
}


template <typename PointT>
void project_cloud_eigen(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, const Eigen::Vector3f& translation_vector, const Eigen::Matrix3f& rotation_matrix, const Eigen::Matrix3f& intrinsics, const Eigen::VectorXf distortions, std::vector<cv::Point2f>& imagePoints)
//void project_cloud_eigen(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, const Eigen::Vector3f& translation_vector, const Eigen::Matrix3f& rotation_matrix, const Eigen::Matrix3f& intrinsics, const Eigen::VectorXf distortions, Eigen::MatrixX2f& imagePoints)
{
    cv::Mat tvec, rmat, rvec, rotmat, K, D;
    //std::vector<cv::Point2f> cv_pts;

    cv::eigen2cv(translation_vector, tvec);
    cv::eigen2cv(rotation_matrix, rmat);
    cv::Rodrigues(rmat, rvec);
    cv::eigen2cv(intrinsics, K);
    cv::eigen2cv(distortions, D);

    project_cloud(cloud, tvec, rvec, K, D, imagePoints);

    //imagePoints.resize(2, cv_pts.size());
    //for (int k = 0; k < cv_pts.size(); k++)
    //{
        //imagePoints(0, k) = cv_pts[k].x;
        //imagePoints(1, k) = cv_pts[k].y;
    //}
}

template<typename PointT>
void filter_lidar_cloud(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, boost::shared_ptr<pcl::PointCloud<PointT> > output_cloud, std::vector<int>& filtered_indices)
{
    filtered_indices.clear();

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, std::numeric_limits<float>::max());
    pass.filter(filtered_indices);
    pass.filter(*output_cloud);
}
