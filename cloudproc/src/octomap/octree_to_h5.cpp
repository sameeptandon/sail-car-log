#include <boost/shared_ptr.hpp>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include "parameters.h"
#include "utils/hdf_utils.h"

int main(int argc, char** arv)
{
    params().initialize();

    // Load the octomap

    boost::shared_ptr<octomap::ColorOcTree> octree((octomap::ColorOcTree*)octomap::ColorOcTree::read(params().color_octomap_file));
    std::cout << "Loaded octree of size " << octree->size() << std::endl;
    if (!octree->size())
        return 1;

    Eigen::MatrixXf octree_mat(octree->size(), 8);

    int k = 0;
    for(octomap::ColorOcTree::leaf_iterator it=octree->begin_leafs(), end=octree->end_leafs(); it != end; ++it)
    {
        octomap::point3d p = it.getCoordinate();
        octomap::ColorOcTreeNode::Color c = it->getColor();
        octree_mat.row(k) << p.x(), p.y(), p.z(), it.getSize(), it->getOccupancy(), c.r, c.g, c.b;
        k++;
    }

    MatrixXfRowMajor row_major(octree_mat);

    H5::H5File file(params().color_octomap_h5_file, H5F_ACC_TRUNC);
    write_hdf_dataset(file, "/octree", row_major, H5::PredType::NATIVE_FLOAT);
    file.close();
}
