#include <string>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "utils/ldr_utils.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options
{
  std::string ldr_file;
  std::string pcd_file;
  po::options_description desc;
};


int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc = opts.desc;
  desc.add_options()("help", "Produce help message.");
  desc.add_options()
    ("ldr", po::value<std::string>(&opts.ldr_file)->required(), "ldr file with points dataset")
    ("pcd", po::value<std::string>(&opts.pcd_file)->required(), "path to output pcd file")
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


// FIXME Use template so can pass build RGB point clouds as well
void matrix_to_cloud(Eigen::MatrixXf& points, pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    cloud.clear();
    cloud.is_dense = false;
    cloud.points.resize(points.rows());
    for (int r=0; r < points.rows(); r++)
    {
        pcl::PointXYZI& p = cloud.at(r);
        p.x = points(r, 0);
        p.y = points(r, 1);
        p.z = points(r, 2);
        p.intensity = points(r, 3);
    }
}


int main(int argc, char** argv)
{
    Options opts;
    if (options(argc, argv, opts))
        return 1;

    PCL_INFO("Loading ldr file: %s\n", opts.ldr_file.c_str());

    Eigen::MatrixXf points;
    readLDRFile(opts.ldr_file, points);
    std::cout << points.rows() << "x" << points.cols() << std::endl;

    pcl::PointCloud<pcl::PointXYZI> cloud;
    matrix_to_cloud(points, cloud);
    cloud.width = points.rows();
    cloud.height = 1;

    PCL_INFO("Writing PCD file: %s\n", opts.pcd_file.c_str());
    //pcl::io::savePCDFileASCII(opts.pcd_file, cloud);
    pcl::io::savePCDFileBinary(opts.pcd_file, cloud);

    return 0;
}
