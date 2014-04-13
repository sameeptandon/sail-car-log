#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include "point_defs.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options
{
  std::string src_pcd_file;
  std::string out_pcd_file;
  int k_search;
  po::options_description desc;
};

int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc = opts.desc;
  desc.add_options()("help", "Produce help message.");
  desc.add_options()
    ("src_pcd", po::value<std::string>(&opts.src_pcd_file)->required(), "path to source pcd file")
    ("out_pcd", po::value<std::string>(&opts.out_pcd_file)->required(), "path to output pcd file")
    ("k", po::value<int>(&opts.k_search)->default_value(30), "k to use for neighbor search")
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
    Options opts;
    if (options(argc, argv, opts))
        return 1;

    // Load stuff

    PCL_INFO("Loading src_pd: %s\n", opts.src_pcd_file.c_str());
    pcl::PointCloud<PointXYZ>::Ptr src_cloud(new pcl::PointCloud<PointXYZ>());
    if (pcl::io::loadPCDFile(opts.src_pcd_file, *src_cloud) < 0)
    {
        std::cout << "Error loading input point cloud " << opts.src_pcd_file << std::endl;
        return -1;
    }

    // Do normal estimation

    PointCloudWithNormals::Ptr out_cloud(new PointCloudWithNormals);

    pcl::NormalEstimation<PointXYZ, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(opts.k_search);

    norm_est.setInputCloud(src_cloud);
    // Only outputs the normals
    norm_est.compute (*out_cloud);
    // This copies over the XYZ coordinates
    pcl::copyPointCloud (*src_cloud, *out_cloud);

    // Save cloud with normals

    pcl::io::savePCDFileBinary(opts.out_pcd_file, *out_cloud);

    return 0;
}
