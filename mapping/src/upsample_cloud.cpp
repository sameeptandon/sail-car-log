#include <iostream>
#include <string>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>

#include "point_defs.h"
#include "utils/ldr_utils.h"

// TODO Examine whether upsampled intensity information makes sense


namespace po = boost::program_options;

struct Options
{
    std::string src_ldr_file;
    std::string out_pcd_file;
    std::string out_ldr_file;
    po::options_description desc;
};

int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc = opts.desc;
  desc.add_options()("help", "Produce help message.");
  desc.add_options()
    ("src_ldr", po::value<std::string>(&opts.src_ldr_file)->required(), "path to source pcd file")
    ("out_pcd", po::value<std::string>(&opts.out_pcd_file)->default_value(""), "path to output pcd file")
    ("out_ldr", po::value<std::string>(&opts.out_ldr_file)->default_value(""));
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


void ldr_to_cloud(const Eigen::MatrixXf& ldr_data, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    cloud.clear();
    cloud.is_dense = false;
    cloud.points.resize(ldr_data.rows());
    for (int r=0; r < ldr_data.rows(); r++)
    {
        pcl::PointXYZ& p = cloud.at(r);
        p.x = ldr_data(r, 0);
        p.y = ldr_data(r, 1);
        p.z = ldr_data(r, 2);
    }
}


int main(int argc, char** argv)
{
    Options opts;
    if (options(argc, argv, opts))
        return 1;

    // Load stuff

    Eigen::MatrixXf ldr_data;
    readLDRFile(opts.src_ldr_file, ldr_data);
    std::cout << "rows: " << ldr_data.rows() << " cols: " << ldr_data.cols() << std::endl;
    std::cout << ldr_data.block<10, 6>(0, 0) << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    ldr_to_cloud(ldr_data, *src_cloud);
    std::cout << "Cloud size: " << src_cloud->size() << std::endl;

    /*
    PCL_INFO("Loading src_pcd: %s\n", opts.src_pcd_file.c_str());
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(opts.src_pcd_file, *src_cloud) < 0)
    {
        std::cout << "Error loading input point cloud " << opts.src_pcd_file << std::endl;
        return -1;
    }
    */

    // Do upsampling 

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls_upsampling;

    // Set up KD-tree
    // TODO Weight intensity
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;

    // Set parameters
    // FIXME PARAMS
    mls_upsampling.setInputCloud(src_cloud);
    mls_upsampling.setComputeNormals (true);
    mls_upsampling.setPolynomialFit (true);
    mls_upsampling.setSearchMethod (tree);
    mls_upsampling.setSearchRadius (0.25);
    //mls_upsampling.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::VOXEL_GRID_DILATION);
    //mls_upsampling.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
    mls_upsampling.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::NONE);
    mls_upsampling.setUpsamplingRadius (0.25);
    mls_upsampling.setUpsamplingStepSize (0.125);
    //mls_upsampling.setDilationVoxelSize(0.25);
    //mls_upsampling.setDilationIterations(1);
    mls_upsampling.setPointDensity(1);

    NormalCloud::Ptr out_cloud(new NormalCloud());
    std::cout << "Running upsampling" << std::endl;
    mls_upsampling.process(*out_cloud);
    std::cout << "Finished upsampling" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*out_cloud, *out_cloud_xyz);

    // TODO For each point in upsampled cloud, assign it the
    // time, laser num, intensity of the closet point in
    // the original cloud
    // TODO Will have to read in original ldr file to do this

    //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2;

    Eigen::MatrixXf out_matrix(out_cloud_xyz->size(), 6);
    for (int k = 0; k < out_cloud_xyz->size(); k++)
    {
        pcl::PointXYZ pt = out_cloud_xyz->at(k);
        // FIXME Fill in with intensity, laser
        out_matrix.row(k) << (float)pt.x, (float)pt.y, (float)pt.z, 0.0f, 0.0f, 0.0f;
    }

    // Save upsampled cloud

    if (opts.out_pcd_file != "")
        pcl::io::savePCDFileBinary(opts.out_pcd_file, *out_cloud);

    // Save ldr file

    if (opts.out_ldr_file != "")
        writeLDRFile(opts.out_ldr_file, out_matrix);

    PCL_INFO("Upsampled from %d to %d points\n", src_cloud->size(), out_cloud->size());

    return 0;
}
