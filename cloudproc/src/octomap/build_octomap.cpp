#include <string>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include "point_defs.h"
#include <pcl/io/pcd_io.h>

#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include <octomap/OccupancyOcTreeBase.h>

#include "utils/path_utils.h"
#include "utils/cloud_utils.h"


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
    std::string pcd_dir = argv[1];
    std::string out_file = argv[2];

    // TODO Resolution parameter
    float OCTREE_RES = 0.5;
    // TODO ColorOcTree

    boost::shared_ptr<octomap::OcTree> octree(new octomap::OcTree(OCTREE_RES));


    std::vector<std::string> pcd_paths;
    get_numbered_files(pcd_dir, "(\\d+).pcd", pcd_paths);

    octomap::point3d sensor_origin;
    sensor_origin(0) = 0;
    sensor_origin(1) = 0;
    sensor_origin(2) = 0;

    int k = -1;
    int step = 5;
    BOOST_FOREACH(std::string pcd_path, pcd_paths)
    {
        k++;
        if (k % step != 0)
            continue;
        if (k > 50)
            break;

        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        octomap::Pointcloud octomap_cloud;
        //std::cout << "Reading " << pcd_path << std::endl;
        load_cloud(pcd_path, src_cloud);
        //std::cout << "Loaded " << pcd_path << std::endl;
        pcl_to_octomap(src_cloud, octomap_cloud);
        std::cout << octomap_cloud.size() << std::endl;

        octree->insertPointCloud(octomap_cloud, sensor_origin);

    }

    octree->write(out_file);

    return 0;
}
