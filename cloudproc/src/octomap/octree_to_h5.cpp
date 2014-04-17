#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include "parameters.h"
#include "utils/hdf_utils.h"

namespace po = boost::program_options;

struct Options
{
    std::string octree_file;
    std::string h5_file;
    bool color;
    bool debug;

    po::options_description desc;
};


int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc = opts.desc;
  desc.add_options()("help", "Produce help message.");
  desc.add_options()
    ("octree", po::value<std::string>(&opts.octree_file)->required(), "octomap file to use")
    ("h5", po::value<std::string>(&opts.h5_file)->required(), "h5 file to output to")
    ("color", po::bool_switch(&opts.color)->default_value(false), "whether we're converting a color octomap or not")
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


int main(int argc, char** argv)
{
    Options opts;
    if (options(argc, argv, opts))
        return 1;

    params().initialize();

    // Load the octomap

    int k = 0;
    Eigen::MatrixXf octree_mat;

    if (opts.color)
    {
        boost::shared_ptr<octomap::ColorOcTree> octree((octomap::ColorOcTree*)octomap::ColorOcTree::read(opts.octree_file));

        octree_mat = Eigen::MatrixXf(octree->size(), 8);
        for(octomap::ColorOcTree::leaf_iterator it=octree->begin_leafs(), end=octree->end_leafs(); it != end; ++it)
        {
            octomap::point3d p = it.getCoordinate();
            octomap::ColorOcTreeNode::Color c = ((octomap::ColorOcTree::leaf_iterator)it)->getColor();
            octree_mat.row(k) << p.x(), p.y(), p.z(), it.getSize(), it->getOccupancy(), c.r, c.g, c.b;
            k++;
        }
    }
    else
    {
        boost::shared_ptr<octomap::OcTree> octree((octomap::OcTree*)octomap::OcTree::read(opts.octree_file));
        octree_mat = Eigen::MatrixXf(octree->size(), 5);
        for(octomap::OcTree::leaf_iterator it=octree->begin_leafs(), end=octree->end_leafs(); it != end; ++it)
        {
            octomap::point3d p = it.getCoordinate();
            octree_mat.row(k) << p.x(), p.y(), p.z(), it.getSize(), it->getOccupancy();
            k++;
        }
    }

    MatrixXfRowMajor row_major(octree_mat);

    H5::H5File file(opts.h5_file, H5F_ACC_TRUNC);
    write_hdf_dataset(file, "/octree", row_major, H5::PredType::NATIVE_FLOAT);
    file.close();
}
