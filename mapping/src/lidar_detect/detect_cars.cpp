#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <limits>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>

#include "parameters.h"
#include "support_plane.h"
#include "utils/cloud_utils.h"
#include "utils/cv_utils.h"
#include "../videoreader/VideoReader.h"


typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

namespace po = boost::program_options;
namespace fs = boost::filesystem;


struct Options
{
  std::string out_file;
  bool debug;
  po::options_description desc;
};


void transform_from_camera_to_lidar(CloudPtr cloud_in_camera_frame, CloudPtr cloud_in_lidar_frame)
{
  Eigen::Matrix4f rot_to_l_from_c = params().rot_to_c_from_l.inverse();
  Eigen::Matrix4f trans_from_c_to_l = params().trans_from_l_to_c.inverse();
  pcl::transformPointCloud(*cloud_in_camera_frame, *cloud_in_lidar_frame, rot_to_l_from_c);
  pcl::transformPointCloud(*cloud_in_lidar_frame, *cloud_in_lidar_frame, trans_from_c_to_l);
}


void transform_from_lidar_to_camera(CloudPtr cloud_in_lidar_frame, CloudPtr cloud_in_camera_frame)
{
  pcl::transformPointCloud(*cloud_in_lidar_frame, *cloud_in_camera_frame, params().trans_from_l_to_c);
  pcl::transformPointCloud(*cloud_in_camera_frame, *cloud_in_camera_frame, params().rot_to_c_from_l);
}


void transform_from_camera_to_lidar(Point& point_in_camera_frame, Point& point_in_lidar_frame)
{
  Eigen::Matrix4f rot_to_l_from_c = params().rot_to_c_from_l.inverse();
  Eigen::Matrix4f trans_from_c_to_l = params().trans_from_l_to_c.inverse();
  Eigen::Transform<float, 3, Eigen::Affine> rot(rot_to_l_from_c);
  Eigen::Transform<float, 3, Eigen::Affine> trans(trans_from_c_to_l);
  point_in_lidar_frame = pcl::transformPoint(point_in_camera_frame, rot);
  point_in_lidar_frame = pcl::transformPoint(point_in_lidar_frame, trans);
}


void transform_from_lidar_to_camera(Point& point_in_lidar_frame, Point& point_in_camera_frame)
{
  Eigen::Transform<float, 3, Eigen::Affine> trans_from_l_to_c(params().trans_from_l_to_c);
  Eigen::Transform<float, 3, Eigen::Affine> rot_to_c_from_l(params().rot_to_c_from_l);
  point_in_camera_frame = pcl::transformPoint(point_in_lidar_frame, trans_from_l_to_c);
  point_in_camera_frame = pcl::transformPoint(point_in_camera_frame, rot_to_c_from_l);
}


int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc = opts.desc;
  desc.add_options()("help", "Produce help message.");
  desc.add_options()
    ("out", po::value<std::string>(&opts.out_file)->required(), "path to output video file")
    ("debug", po::bool_switch(&opts.debug)->default_value(false), "debug mode")
    ;
  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  po::notify(vm);

  return 0;
}


int main(int argc, char** argv)
{
  params().initialize();
  int start = params().start;
  int step = params().step;
  int end = params().end;

  Options opts;
  if (options(argc, argv, opts))
    return 1;

  std::vector<float> filter_range = params().filter_range;
  assert (filter_range.size() == 6);
  float filter_range_min_x = filter_range[0]; float filter_range_max_x = filter_range[1];
  float filter_range_min_y = filter_range[2]; float filter_range_max_y = filter_range[3];
  float filter_range_min_z = filter_range[4]; float filter_range_max_z = filter_range[5];

  std::vector<float> g = params().g;
  Eigen::Vector3f gvec(g[0], g[1], g[2]);
  gvec.normalize();

  // Initialize reader

  VideoReader reader(params().dset_dir, params().dset_avi);
  if (start > 0)
      reader.setFrame(start - 1);

  // Set up a writer as well

  cv::VideoWriter writer;
  int fps = 30;
  //writer.open(argv[1], reader.getCodecType(), fps, cv::Size(reader.getFrameWidth(), reader.getFrameHeight()), true);
  writer.open(opts.out_file, CV_FOURCC( 'M', 'J', 'P', 'G' ), fps, cv::Size(reader.getFrameWidth(), reader.getFrameHeight()), true);
  if (!writer.isOpened())
    std::cout << "Could not open " << opts.out_file << " to write video" << std::endl;

  cv::Mat frame;
  std::vector<cv::Point2f> pixels;
  std::vector<cv::Point2f> filtered_pixels;
  std::vector<cv::Point2f> corner_pixels;

  for (int k = start; k < end; k += step)
  {
    int num_steps = (k - start) / step;

    bool success = reader.getNextFrame(frame);
    if (!success)
    {
        std::cout << "Reached end of video before expected" << std::endl;
        return 1;
    }

    // Load in pcd file

    std::string pcd_file = (fs::path(params().pcd_dir) / fs::path((boost::format("%1%.pcd") % (num_steps)).str())).string();
    CloudPtr cloud (new Cloud);
    std::cout << "loading " << pcd_file << std::endl;
    load_cloud(pcd_file, cloud);

    // Remove NaN's from point cloud

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // Filter away everything outside of range
    // Keep this before fitModel

    filter_field(cloud, "x", filter_range_min_x, filter_range_max_x);
    filter_field(cloud, "y", filter_range_min_y, filter_range_max_y);
    filter_field(cloud, "z", filter_range_min_z, filter_range_max_z);

    // Segment plane using RANSAC and filter away points below plane

    SupportPlane<pcl::PointXYZ> support_plane;

    support_plane.setInputCloud(cloud->makeShared());
    int fit_success = support_plane.fitModel(params().inlier_dist_thresh, params().highest_plane);
    if (fit_success == -1)
    {
      PCL_ERROR("Could not find a supporting plane in scene\n");
      return (-1);
    }

    // Points above plane

    CloudPtr cloud_filtered(new Cloud);
    support_plane.filterInliers(cloud_filtered);

    //support_plane.computeHull();
    //support_plane.pointsAboveHull(cloud, cloud_filtered);
    support_plane.pointsAbovePlane(cloud_filtered, cloud_filtered);
    // FIXME
    if (cloud_filtered->size() == 0)
    {
      std::cout << "cloud " << num_steps << " has no points after filtering..." << std::endl;
    }
    else
    {
      // Transform to camera_t
      transform_from_lidar_to_camera(cloud_filtered, cloud_filtered);
      // Only points in front of the camera
      filter_field(cloud_filtered, "z", 0, std::numeric_limits<float>::max());

      // Perform clustering

      std::vector<CloudPtr> clusters;
      extract_clusters_euclidean(cloud_filtered, clusters, params().obj_cluster_tol, params().obj_min_cluster_size, params().obj_max_cluster_size);

      std::cout << "Extracted " << clusters.size() << " clusters" << std::endl;

      // TODO Draw points and bounding boxes on image

      for (int k = 0; k < clusters.size(); k++)
      {
        // Project the point onto the plane (for occluded cars)
        // Have to first transform the cluster back into lidar coordinates

        pcl::PointXYZ min_pt, max_pt;
        transform_from_camera_to_lidar(clusters[k], clusters[k]);
        pcl::getMinMax3D(*(clusters[k]), min_pt, max_pt);
        support_plane.projectPointOnPlane(min_pt, min_pt);
        transform_from_lidar_to_camera(clusters[k], clusters[k]);

        transform_from_lidar_to_camera(min_pt, min_pt);
        transform_from_lidar_to_camera(max_pt, max_pt);

        // Determine 3d bounding box

        CloudPtr corners_cloud(new Cloud);
        get_box_corners(min_pt, max_pt, corners_cloud);
        project_cloud_eigen(corners_cloud, Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(),
                params().intrinsics, params().distortions, corner_pixels);
        draw_bbox_3d_from_corner_pixels(corner_pixels, cv::Scalar(0, 0, 255), frame, 2);

        // Project point cloud and draw 2D bounding box

        project_cloud_eigen(clusters[k], Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(),
                params().intrinsics, params().distortions, pixels);

        std::vector<int> filtered_pixel_indices;
        filter_pixels(pixels, frame, filtered_pixels, filtered_pixel_indices);
        if (filtered_pixels.size() > params().obj_min_cluster_size)
            set_pixel_colors(filtered_pixels, cv::Vec3b(255, 0, 0), frame, 2);

            // Don't use filtered here
            bbox box = compute_bbox(pixels);
            if (box.xy_ratio() > 0.1) // FIXME PARAM
            {
                draw_bbox(box, cv::Scalar(0, 255, 0), frame, 3);
            }
      }
    }

    if (opts.debug)
    {
      support_plane.visualize(cloud, support_plane.hull_cloud_,
          support_plane.getNormal());
      return 0;
    }


    // Write video

    writer.write(frame);

    // Skip

    success = reader.skip(step - 1);
    if (!success)
    {
      std::cout << "Reached end of video before expected" << std::endl;
      return 1;
    }
  }

  return 0;
}
