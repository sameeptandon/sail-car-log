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
#include <octomap/ColorOcTree.h>

#include "utils/path_utils.h"
#include "utils/cloud_utils.h"
#include "utils/hdf_utils.h"

#include "parameters.h"

namespace fs = boost::filesystem;

template <typename PointT>
void pcl_to_octomap(boost::shared_ptr<pcl::PointCloud<PointT> > src_cloud, octomap::Pointcloud& octomap_cloud)
{
    octomap_cloud.clear();
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

    int count = params().count;

    std::string transforms_dir = params().h5_dir;

    std::vector<std::string> pcd_paths;
    // Read transforms
    std::vector<std::string> transform_paths;

    get_range_files(params().color_clouds_dir, 0, 1, count, "%1%.pcd", pcd_paths);
    get_range_files(transforms_dir, 0, 1, count, "%1%.transform", transform_paths);

    assert(pcd_paths.size() == transform_paths.size());

    // Transformations

    octomap::point3d sensor_origin;
    MatrixXfRowMajor transform;
    Eigen::Vector3f  init_pos;

    // Build the octomap

    boost::progress_display show_progress(pcd_paths.size());

    boost::shared_ptr<octomap::ColorOcTree> octree(new octomap::ColorOcTree(params().color_octree_res));
    octree->setProbHit(params().prob_hit);
    octree->setProbMiss(params().prob_miss);
    octree->setClampingThresMax(params().clamping_thres_max);
    octree->setClampingThresMin(params().clamping_thres_min);
    boost::shared_ptr<octomap::ColorOcTree> octree_centered(new octomap::ColorOcTree(*octree));

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

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        load_cloud(pcd_path, src_cloud);

        octomap::Pointcloud octomap_cloud;
        pcl_to_octomap(src_cloud, octomap_cloud);
        octree->insertPointCloud(octomap_cloud, sensor_origin);
        BOOST_FOREACH(pcl::PointXYZRGB pt, src_cloud->points)
        {
            octomap::ColorOcTreeNode* node = octree->search(pt.x, pt.y, pt.z);
            if (node)
                node->setColor(pt.r, pt.g, pt.b);
        }

        // Following is for octovis so the map is close to centered
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr centered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        if (k == 0)
            init_pos = transform_copy.block<3, 1>(0, 3);
        sensor_origin(0) -= init_pos(0);
        sensor_origin(1) -= init_pos(1);
        sensor_origin(2) -= init_pos(2);
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T(0, 3) -= init_pos(0);
        T(1, 3) -= init_pos(1);
        T(2, 3) -= init_pos(2);
        pcl::transformPointCloud(*src_cloud, *centered_cloud, T);

        pcl_to_octomap(centered_cloud, octomap_cloud);
        octree_centered->insertPointCloud(octomap_cloud, sensor_origin);
        BOOST_FOREACH(pcl::PointXYZRGB pt, centered_cloud->points)
        {
            octomap::ColorOcTreeNode* node = octree_centered->search(pt.x, pt.y, pt.z);
            if (node)
                node->setColor(pt.r, pt.g, pt.b);
        }

        ++show_progress;
    }

    std::cout << "Writing octree of size " << octree->size() << std::endl;
    octree->write(params().color_octomap_file);
    octree_centered->writeBinary(params().centered_color_octomap_file);

    return 0;
}
