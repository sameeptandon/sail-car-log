#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Viewer using PCLVisualizer that can display axes
// More features pending

// FIXME PARAM
bool color = false;
typedef pcl::PointXYZI PointT;

int main(int argc, char** argv)
{
    std::string pcd_path = argv[1];

    boost::shared_ptr<pcl::PointCloud<PointT> > cloud(new pcl::PointCloud<PointT>());

    if (pcl::io::loadPCDFile(pcd_path, *cloud) < 0)
    {
        std::cout << "Error loading input point cloud " << pcd_path << std::endl;
        return 1;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer("view_pcd"));
    if (color)  // FIXME
    {
        //pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb (cloud);
        //vis->addPointCloud<PointT>(cloud, rgb);

    }
    else
    {
        vis->addPointCloud<PointT>(cloud);
    }
    vis->addCoordinateSystem(100.0);
    vis->spin();

    return (0);
}
