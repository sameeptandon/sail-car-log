#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <limits>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

#include "parameters.h"
#include "support_plane.h"
#include "utils/cloud_utils.h"
#include "utils/cv_utils.h"
#include "../videoreader/VideoReader.h"


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
      pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, params().trans_from_l_to_c);
      pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, params().rot_to_c_from_l);
      // Only points in front of the camera
      filter_field(cloud_filtered, "z", 0, std::numeric_limits<float>::max());

      // Perform clustering

      std::vector<CloudPtr> clusters;
      extract_clusters_euclidean(cloud_filtered, clusters, params().obj_cluster_tol, params().obj_min_cluster_size, params().obj_max_cluster_size);

      std::cout << "Extracted " << clusters.size() << " clusters" << std::endl;

      // TODO Draw points and bounding boxes on image

      for (int k = 0; k < clusters.size(); k++)
      {
        project_cloud_eigen(clusters[k], Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(),
                params().intrinsics, params().distortions, pixels);

        std::vector<int> filtered_pixel_indices;
        filter_pixels(pixels, frame, filtered_pixels, filtered_pixel_indices);
        if (filtered_pixels.size() > 0)
            set_pixel_colors(filtered_pixels, cv::Vec3b(255, 0, 0), frame, 2);
            draw_bbox_from_pixels(filtered_pixels, cv::Scalar(0, 255, 0), frame, 3);
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
