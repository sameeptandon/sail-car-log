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
    std::string pcd_dir;
    std::string out_dir;
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
    ("pcd_dir", po::value<std::string>(&opts.pcd_dir)->required(), "path containing pcd files")
    ("out_dir", po::value<std::string>(&opts.out_dir)->default_value(""), "path to output files, defaults to pcd_dir")
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

    // Read in all the PCD files

    fs::path pcd_path(opts.pcd_dir);
    assert(fs::exists(pcd_path) && fs::is_directory(pcd_path));

    fs::directory_iterator end_iter;
    typedef std::multimap<int, fs::path> int_path_map;
    int_path_map pcd_paths;

    boost::smatch match;
    boost::regex re("(\\d+).pcd");
    for (fs::directory_iterator dir_iter(pcd_path); dir_iter != end_iter; ++dir_iter)
    {
        if (fs::is_regular_file(dir_iter->status()))
        {
            fs::path p = *dir_iter;
            std::string leaf = p.leaf().string();
            if (boost::regex_match(leaf, match, re))
            {
                std::string object_id(match[1].first, match[1].second);
                int obj_id = st2num<int>(object_id.c_str());
                pcd_paths.insert(int_path_map::value_type(obj_id, *dir_iter));
            }
        }
    }

    PointCloudWithNormals::Ptr full_cloud(new PointCloudWithNormals());

    std::vector<Eigen::VectorXf> transforms;
    Eigen::VectorXf fitness_scores(pcd_paths.size() - 1);

    int k = 0;
    for (int_path_map::iterator iter = pcd_paths.begin(); iter != pcd_paths.end(); ++iter)
    {
        std::string pcd_file = iter->second.string();
        std::cout << "reading " << pcd_file << std::endl;
        if (full_cloud->size() == 0)
        {
            // Load the initial cloud
            load_cloud(pcd_file, full_cloud);
            continue;
        }

        PointCloudWithNormals::Ptr cloud(new PointCloudWithNormals());
        load_cloud(pcd_file, cloud);

        // Align

        Eigen::Matrix4f transform;
        PointCloudWithNormals::Ptr aligned_cloud(new PointCloudWithNormals());
        float score = pair_align(cloud, full_cloud, aligned_cloud, transform, opts.icp_iters, opts.max_dist);

        *full_cloud += *aligned_cloud;

        transforms.push_back(Eigen::Map<Eigen::VectorXf>(transform.data(), transform.size()));
        fitness_scores(k) = score;
        k++;
    }

    Eigen::MatrixXf transforms_mat(transforms.size(), 16);
    for (int k = 0; k < transforms.size(); k++)
        transforms_mat.row(k) << transforms.at(k).transpose();

    // Save the transforms and fitness scores in h5 file

    fs::path out_path = pcd_path;
    if (opts.out_dir != "")
        out_path = opts.out_dir;

    H5::H5File file((out_path / "icp.h5").string(), H5F_ACC_TRUNC);
    std::cout << "writing transforms" << std::endl;
    write_hdf_dataset(file, "/transforms", transforms_mat, H5::PredType::NATIVE_FLOAT);
    std::cout << "writing fitness_scores" << std::endl;
    write_hdf_dataset(file, "/fitness_scores", fitness_scores, H5::PredType::NATIVE_FLOAT);
    file.close();

    // Save the merged point cloud

    std::cout << "saving pcd" << std::endl;
    pcl::io::savePCDFile((out_path / "merged.pcd").string(), *full_cloud, true);

    return 0;
}
