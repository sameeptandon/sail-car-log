#include <octomap/octomap.h>

#include "utils/cloud_utils.h"
#include "parameters.h"

typedef pcl::PointXYZRGB PointType;

int main(int argc, char** argv)
{
    std::string merged_cloud_file = argv[1];
    std::string static_cloud_file = argv[2];
    std::string dynamic_cloud_file = argv[3];

    params().initialize();

    std::cout << "Loading " << params().octomap_file << std::endl;
    boost::shared_ptr<octomap::OcTree> octree((octomap::OcTree*)octomap::OcTree::read(params().octomap_file));
    std::cout << "Loaded octree of size " << octree->size() << std::endl;

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    load_cloud(merged_cloud_file, cloud);

    pcl::PointCloud<PointType>::Ptr static_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr dynamic_cloud(new pcl::PointCloud<PointType>());

    for (int k = 0; k < cloud->size(); k++)
    {
        PointType pt = cloud->at(k);

        octomap::OcTreeNode* node = octree->search(pt.x, pt.y, pt.z);
        octomap::OcTreeNode* node_below = octree->search(pt.x, pt.y, pt.z - params().octree_res);

        // FIXME PARAM
        // FIXME ALso filter out points way below
        if ((node && octree->isNodeOccupied(node)))
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
