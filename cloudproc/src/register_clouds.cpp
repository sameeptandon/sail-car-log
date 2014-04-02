#include <iostream>
#include <string>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "point_defs.h"
#include "utils/hdf_utils.h"


namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options
{
    std::string pcd_tgt;
    std::string pcd_src;
    std::string h5_file;
    int icp_iters;
    float max_dist;
    po::options_description desc;
};

int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc = opts.desc;
  desc.add_options()("help", "Produce help message.");
  desc.add_options()
    ("pcd_tgt", po::value<std::string>(&opts.pcd_tgt)->required(), "pcd of cloud to align to")
    ("pcd_src", po::value<std::string>(&opts.pcd_src)->required(), "pcd of cloud to align")
    ("h5_file", po::value<std::string>(&opts.h5_file)->required(), "h5 file to alignment information to")
    ("icp_iters", po::value<int>(&opts.icp_iters)->default_value(5), "number of ICP iterations to run")
    ("max_dist", po::value<float>(&opts.max_dist)->default_value(1.0), "maximum ICP correspondence distance")
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

// TODO Move to utils file

template <typename T>
T st2num ( const std::string &Text )
{
     std::stringstream ss(Text);
     T result;
     return ss >> result ? result : 0;
}

float pair_align (const PointCloudWithNormals::Ptr cloud_src, const PointCloudWithNormals::Ptr tgt_cloud, PointCloudWithNormals::Ptr aligned_cloud, Eigen::Matrix4f &transform, int icp_iters, float max_dist)
{
    // TODO Determine whether need to set a custom point representation

    // Align

    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-6);  // Change in transformation for (convergence) // PARAM
    reg.setMaxCorrespondenceDistance (max_dist);
    reg.setMaximumIterations (2);  // Maximum iterations to run internal optimization // PARAM
    reg.setInputTarget(tgt_cloud);

    Eigen::Matrix4f T_i = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f prev;


    PointCloudWithNormals::Ptr src_cloud(new PointCloudWithNormals());
    pcl::copyPointCloud (*cloud_src, *src_cloud);

    float prev_fitness_score = std::numeric_limits<float>::max();
    for (int i = 0; i < icp_iters; ++i)
    {
        reg.setInputSource(src_cloud);
        reg.align(*aligned_cloud);
        src_cloud = aligned_cloud;

        // Accumulate transformation
        T_i = reg.getFinalTransformation () * T_i;

        // If the difference between this transformation and the previous one
        // is smaller than the threshold, refine the process by reducing
        // the maximal correspondence distance
        if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);


        prev = reg.getLastIncrementalTransformation ();

        PCL_INFO("\titer: %d, fitness score: %f\n", i, reg.getFitnessScore());

        if (reg.getFitnessScore() > prev_fitness_score)
        {
            prev_fitness_score = reg.getFitnessScore();
            break;
        }
        prev_fitness_score = reg.getFitnessScore();
    }

    transform = T_i.inverse();
    return reg.getFitnessScore();
}

void load_cloud(std::string pcd_path, PointCloudWithNormals::Ptr cloud)
{
    if (pcl::io::loadPCDFile(pcd_path, *cloud) < 0)
    {
        std::cout << "Error loading input point cloud " << pcd_path << std::endl;
        throw;
    }
}

int main(int argc, char** argv)
{
    Options opts;
    if (options(argc, argv, opts))
        return 1;

    // Read in the PCD files

    PointCloudWithNormals::Ptr src_cloud(new PointCloudWithNormals());
    PointCloudWithNormals::Ptr tgt_cloud(new PointCloudWithNormals());
    load_cloud(opts.pcd_src, src_cloud);
    load_cloud(opts.pcd_tgt, tgt_cloud);

    PointCloudWithNormals::Ptr aligned_cloud(new PointCloudWithNormals());
    Eigen::Matrix4f transform;

    float score = pair_align(src_cloud, tgt_cloud, aligned_cloud, transform, opts.icp_iters, opts.max_dist);

    transform.transposeInPlace();  // Since H5 uses row-major

    H5::H5File file(opts.h5_file, H5F_ACC_TRUNC);
    write_hdf_dataset(file, "/transform", transform, H5::PredType::NATIVE_FLOAT);
    write_hdf_attribute(file, "/transform", "fitness_score", &score);
    write_hdf_attribute(file, "/transform", "pcd_src", opts.pcd_src);
    write_hdf_attribute(file, "/transform", "pcd_tgt", opts.pcd_tgt);
    file.close();

    return 0;
}
