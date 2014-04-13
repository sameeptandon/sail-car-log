#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "point_defs.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options
{
  std::string src_pcd_file;
  std::string out_pcd_file;
  float leaf_size;
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
    ("leaf_size", po::value<float>(&opts.leaf_size)->default_value(0.1), "leaf size for voxel grid downsampling")
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
    PointCloud::Ptr src_cloud(new PointCloud());
    if (pcl::io::loadPCDFile(opts.src_pcd_file, *src_cloud) < 0)
    {
        std::cout << "Error loading input point cloud " << opts.src_pcd_file << std::endl;
        return -1;
    }

    // Do downsampling

    PointCloud::Ptr out_cloud(new PointCloud());
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(src_cloud);
    // FIXME Should be an argument
    voxel_grid.setLeafSize(opts.leaf_size, opts.leaf_size, opts.leaf_size);
    voxel_grid.filter(*out_cloud);

    // Save downsampled cloud

    pcl::io::savePCDFileBinary(opts.out_pcd_file, *out_cloud);

    PCL_INFO("Downsampled from %d to %d points\n", src_cloud->size(), out_cloud->size());

    return 0;
}
