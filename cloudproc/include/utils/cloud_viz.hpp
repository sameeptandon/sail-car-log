#include <pcl/visualization/pcl_visualizer.h>

template<typename PointT>
void align_clouds_viz(const boost::shared_ptr<pcl::PointCloud<PointT> > src_cloud, const boost::shared_ptr<pcl::PointCloud<PointT> > tgt_cloud, boost::shared_ptr<pcl::PointCloud<PointT> > aligned_cloud, const pcl::Correspondences& correspondences, bool viz_normals)
{
    pcl::visualization::PCLVisualizer viz("align clouds viz");
    viz.addCoordinateSystem(3.0);

    // Add point clouds and normals

    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(tgt_cloud, 0, 255, 0);
    viz.addPointCloud(tgt_cloud, green, "tgt_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(tgt_cloud, 0, 0, 255);
    viz.addPointCloud(src_cloud, blue, "src_cloud");

    if (viz_normals)
    {
        viz.addPointCloudNormals<PointT>(tgt_cloud, 1, 0.5f, "tgt_cloud_normals", 0);
        viz.addPointCloudNormals<PointT>(src_cloud, 1, 0.5f, "src_cloud_normals", 0);
    }

    // Add correspondences

    if (correspondences.size() > 0)
        viz.addCorrespondences<PointT>(src_cloud, tgt_cloud, correspondences, "correspondences", 0);

    // Add the final aligned cloud

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> red(aligned_cloud, 255, 0, 0);
    viz.addPointCloud(aligned_cloud, red, "aligned_cloud");

    viz.spin();
}

