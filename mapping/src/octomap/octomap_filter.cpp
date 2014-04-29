#include <octomap/octomap.h>

#include "utils/cloud_utils.h"
#include "parameters.h"

int main(int argc, char** argv)
{
    std::string merged_cloud_file = argv[1];
    std::string static_cloud_file = argv[2];
    std::string dynamic_cloud_file = argv[3];

    params().initialize();

    boost::shared_ptr<octomap::OcTree> octree((octomap::OcTree*)octomap::OcTree::read(params().octomap_file));
    std::cout << "Loaded octree of size " << octree->size() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    load_cloud(merged_cloud_file, cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (int k = 0; k < cloud->size(); k++)
    {
        pcl::PointXYZRGB pt = cloud->at(k);

        octomap::OcTreeNode* node = octree->search(pt.x, pt.y, pt.z);
        octomap::OcTreeNode* node_below = octree->search(pt.x, pt.y + params().octree_res, pt.z);

        // FIXME PARAM
        // FIXME ALso filter out points way below
        if ((node && octree->isNodeOccupied(node)) || (node_below && octree->isNodeOccupied(node_below)) || cloud->at(k).y < -5.0)
        {
            static_cloud->push_back(pt);
        }
        else
        {
            dynamic_cloud->push_back(pt);
        }
    }

    pcl::io::savePCDFileBinary(static_cloud_file, *static_cloud);
    pcl::io::savePCDFileBinary(dynamic_cloud_file, *dynamic_cloud);

    std::cout << static_cloud->size() << " static points, " << dynamic_cloud->size() << " dynamic points" << std::endl;

    return 0;
}
