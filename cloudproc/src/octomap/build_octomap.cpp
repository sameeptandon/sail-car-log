#include <string>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/progress.hpp>

#include "point_defs.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include <octomap/OccupancyOcTreeBase.h>

#include "utils/path_utils.h"
#include "utils/cloud_utils.h"
#include "utils/hdf_utils.h"

#include "parameters.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options
{
    std::string pcd_dir;
    std::string transforms_dir;
    std::string out_file;
    float octree_res;

    int start;
    int step;
    int count;

    bool debug;

    po::options_description desc;
};


int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc = opts.desc;
  desc.add_options()("help", "Produce help message.");
  desc.add_options()
    ("pcd_dir", po::value<std::string>(&opts.pcd_dir)->required(), "directory with pcd files")
    ("transforms_dir", po::value<std::string>(&opts.transforms_dir)->required(), "directory with transform files")
    ("out_file", po::value<std::string>(&opts.out_file)->required(), "file to output octree to")
    ("octree_res", po::value<float>(&opts.octree_res)->default_value(0.5), "resolution of the octree")
    ("start", po::value<int>(&opts.start)->default_value(0), "start cloud index")
    ("step", po::value<int>(&opts.step)->default_value(5), "step size between clouds")
    ("max_count", po::value<int>(&opts.count)->default_value(10), "maximum number of clouds to integrate")
    ("debug", po::bool_switch(&opts.debug)->default_value(false), "debug flag")
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


template <typename PointT>
void pcl_to_octomap(boost::shared_ptr<pcl::PointCloud<PointT> > src_cloud, octomap::Pointcloud& octomap_cloud)
{
    octomap::point3d octopt;
    BOOST_FOREACH(PointT pt, src_cloud->points)
    {
        octopt(0) = pt.x;
        octopt(1) = pt.y;
        octopt(2) = pt.z;
        octomap_cloud.push_back(octopt);
    }
}


int main(int argc, char** argv)
{
    params().initialize();

    // Set up paths

    Options opts;
    if (options(argc, argv, opts))
        return 1;
    int start = opts.start;
    int step = opts.step;
    int count = opts.count;

    std::string transforms_dir = opts.transforms_dir;
    std::string out_file = opts.out_file;

    std::vector<std::string> pcd_paths;
    // Read transforms and rpys (don't want to write C++ to convert to rpy)
    std::vector<std::string> transform_paths;

    //get_numbered_files(opts.pcd_dir, "(\\d+).pcd", pcd_paths);
    //get_numbered_files(opts.transforms_dir, "(\\d+).transform", transform_paths);
    get_range_files(opts.pcd_dir, start, step, count, "%1%.pcd", pcd_paths);
    get_range_files(opts.transforms_dir, start, step, count, "%1%.transform", transform_paths);

    assert(pcd_paths.size() == transform_paths.size());

    // Transformations

    octomap::point3d sensor_origin;
    MatrixXfRowMajor transform;
    Eigen::Vector3f  init_pos;

    // Build the octomap

    boost::progress_display show_progress(pcd_paths.size());

    boost::shared_ptr<octomap::OcTree> octree(new octomap::OcTree(opts.octree_res));
    octree->setProbHit(params().prob_hit);
    octree->setProbMiss(params().prob_miss);
    //octree->setOccupancyThres(params().occupancy_thres);
    //octree->setClampingThresMax(params().clamping_thres_max);
    //octree->setClampingThresMin(params().clamping_thres_min);
    //std::cout << "Occupancy threshold: " << octree->getOccupancyThres() << std::endl;

    for (int k = 0; k < pcd_paths.size(); k++)
    {
        // Load point cloud and transforms

        std::string pcd_path = pcd_paths[k];
        std::string transform_path = transform_paths[k];

        if (!fs::exists(transform_path))
        {
            std::cout << transform_path << " does not exist, quitting" << std::endl;
            break;
        }

        H5::H5File transform_file(transform_path, H5F_ACC_RDONLY);
        load_hdf_dataset(transform_file, "transform", transform, H5::PredType::NATIVE_FLOAT);
        Eigen::Matrix4f transform_copy(transform);

        Eigen::Vector4f imu_origin = transform_copy.block<4, 1>(0, 3);
        Eigen::Vector4f lidar_origin = params().T_from_i_to_l * imu_origin;

        sensor_origin(0) = lidar_origin(0);
        sensor_origin(1) = lidar_origin(1);
        sensor_origin(2) = lidar_origin(2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        //std::cout << "Reading " << pcd_path << std::endl;
        load_cloud(pcd_path, src_cloud);

        // Following is for octovis so the map is close to centered

        if (params().center_octomap)
        {
            if (k == 0)
                init_pos = transform_copy.block<3, 1>(0, 3);
            sensor_origin(0) -= init_pos(0);
            sensor_origin(1) -= init_pos(1);
            sensor_origin(2) -= init_pos(2);
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T(0, 3) -= init_pos(0);
            T(1, 3) -= init_pos(1);
            T(2, 3) -= init_pos(2);
            pcl::transformPointCloud(*src_cloud, *src_cloud, T);
        }

        octomap::Pointcloud octomap_cloud;
        pcl_to_octomap(src_cloud, octomap_cloud);
        if (opts.debug)
            std::cout << "cloud size:" <<  octomap_cloud.size() << std::endl;

        octree->insertPointCloud(octomap_cloud, sensor_origin);
        ++show_progress;
    }

    std::cout << "Writing octree of size " << octree->size() << std::endl;
    octree->write(out_file);

    return 0;
}
