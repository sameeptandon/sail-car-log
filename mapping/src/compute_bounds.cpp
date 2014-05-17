#include <limits>
#include <algorithm>

#include <boost/foreach.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>

#include "utils/cloud_utils.h"
#include "parameters.h"
#include "utils/hdf_utils.h"


struct bbox_3d
{
    float x_min; float x_max;
    float y_min; float y_max;
    float z_min; float z_max;

    bbox_3d(float x1, float y1, float z1,
            float x2, float y2, float z2)
            : x_min(x1),
              y_min(y1),
              z_min(z1),
              x_max(x2),
              y_max(y2),
              z_max(z2)
    {
    }

    Eigen::VectorXf toVector()
    {
        Eigen::VectorXf vec(6);
        vec << x_min, y_min, z_min, x_max, y_max, z_max;
        return vec;
    }
};


bbox_3d compute_bbox(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::vector<int>& cloud_indices)
{
    bbox_3d bbox( std::numeric_limits<float>::max(),
                  std::numeric_limits<float>::max(),
                  std::numeric_limits<float>::max(),
                 -std::numeric_limits<float>::max(),
                 -std::numeric_limits<float>::max(),
                 -std::numeric_limits<float>::max());

    /*
    Eigen::Matrix<float, 3, 3> cov;
    Eigen::Matrix<float, 4, 1> centroid;

    pcl::computeMeanAndCovarianceMatrix(*cloud, cloud_indices, cov, centroid);

    bbox_3d bbox(centroid(0) - cov(0, 0),
                 centroid(1) - cov(1, 1),
                 centroid(2) - cov(2, 2),
                 centroid(0) + cov(0, 0),
                 centroid(1) + cov(1, 1),
                 centroid(2) + cov(2, 2));
    */

    BOOST_FOREACH(int ind, cloud_indices)
    {
        pcl::PointXYZ p = cloud->at(ind);
        bbox.x_min = std::min(p.x, bbox.x_min);
        bbox.y_min = std::min(p.y, bbox.y_min);
        bbox.z_min = std::min(p.z, bbox.z_min);
        bbox.x_max = std::max(p.x, bbox.x_max);
        bbox.y_max = std::max(p.y, bbox.y_max);
        bbox.z_max = std::max(p.z, bbox.z_max);
    }

    return bbox;
}


void extract_clusters_euclidean(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::vector<pcl::PointIndices>& cluster_indices, float cluster_tol, int min_cluster_size, int max_cluster_size)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tol);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
}


int main(int argc, char** argv)
{
    std::string pcd_file = argv[1];
    std::string h5_file = argv[2];

    params().initialize();

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    load_cloud(pcd_file, cloud);

    std::cout << "cluster_tol: " << params().cluster_tol << std::endl;
    std::cout << "min_cluster_size: " << params().min_cluster_size << std::endl;
    std::cout << "max_cluster_size: " << params().max_cluster_size << std::endl;
    extract_clusters_euclidean(cloud, cluster_indices,
            params().cluster_tol, params().min_cluster_size, params().max_cluster_size);

    int num_clusters = cluster_indices.size();
    Eigen::MatrixXf bboxes(num_clusters, 6);
    for (int k = 0; k < num_clusters; k++)
    {
        pcl::PointIndices indices = cluster_indices[k];
        bbox_3d bbox = compute_bbox(cloud, indices.indices);
        bboxes.row(k) = bbox.toVector();
    }

    MatrixXfRowMajor row_major(bboxes);
    H5::H5File file(h5_file, H5F_ACC_TRUNC);
    write_hdf_dataset(file, "/bboxes", row_major, H5::PredType::NATIVE_FLOAT);
    file.close();
}
