#include <limits>
#include <algorithm>

#include <boost/foreach.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>

#include "utils/cloud_utils.h"
#include "parameters.h"
#include "utils/hdf_utils.h"


void extract_clusters_euclidean(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::vector<pcl::PointIndices>& cluster_indices, float cluster_tol, int min_cluster_size, int max_cluster_size)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setEpsilon(0.0);
    tree->setInputCloud (cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tol);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
}


void compute3DMedian(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::vector<int>& indices, Eigen::Matrix<float, 4, 1>& median)
{
    std::vector<float> xs;
    std::vector<float> ys;
    std::vector<float> zs;
    BOOST_FOREACH(int ind, indices)
    {
        pcl::PointXYZ pt = cloud.at(ind);
        xs.push_back(pt.x);
        ys.push_back(pt.y);
        zs.push_back(pt.z);
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    std::sort(zs.begin(), zs.end());
    median << xs[xs.size() / 2], ys[ys.size() / 2], zs[zs.size() / 2], 0.0f;
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


    // Store the centers of the clusters for comparison

    int num_clusters = cluster_indices.size();
    Eigen::MatrixXf cluster_centers(num_clusters, 3);
    for (int k = 0; k < num_clusters; k++)
    {
        pcl::PointIndices indices = cluster_indices[k];
        Eigen::Matrix<float, 4, 1> centroid;
        pcl::compute3DCentroid(*cloud, indices.indices, centroid);
        //compute3DMedian(*cloud, indices.indices, centroid);
        cluster_centers.row(k) = centroid.block<3, 1>(0, 0);
    }


    // Store the cluster that each point corresponds to
    Eigen::VectorXi cluster_labels = Eigen::VectorXi::Ones(cloud->size()) * -1;
    // NOTE Assuming number of clusters is < INT_MAX
    for (int i = 0; i < num_clusters; i++)
    {
        for (int j = 0; j < cluster_indices[i].indices.size(); j++)
        {
            cluster_labels(cluster_indices[i].indices[j]) = i;
        }
    }

    H5::H5File file(h5_file, H5F_ACC_TRUNC);
    write_hdf_dataset(file, "/cluster_labels", cluster_labels, H5::PredType::NATIVE_INT);

    MatrixXfRowMajor row_major(cluster_centers);
    write_hdf_dataset(file, "/cluster_centers", row_major, H5::PredType::NATIVE_FLOAT);
    file.close();

    file.close();
}
