#include "cloud_server.h"
#include "utils/path_utils.h"
#include <boost/progress.hpp>


void CloudServer::initialize(const std::string& cloud_dir, const std::string& color_dir, const std::string& transforms_dir, int store_max)
{
    if (initialized)
        return;

    cloud_path = cloud_dir;
    color_path = color_dir;
    transforms_path = transforms_dir;
    max_store = store_max;

    // FIXME Assumes have at least max_store files and k=0; k++ indexed files
    get_range_files(cloud_path, 0, 1, max_store, "%1%.pcd", cloud_files);
    get_range_files(color_path, 0, 1, max_store, "%1%.h5", color_files);
    get_range_files(transforms_path, 0, 1, max_store, "%1%.transform", transform_files);

    std::cout << "Initializing server..." << std::endl;
    boost::progress_display show_progress(max_store);
    for (int k = 0; k < max_store; k++)
    {
        pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>());
        load_cloud(cloud_files[k], cloud);
        clouds.push_back(cloud);

        Eigen::MatrixXi color = Eigen::MatrixXi::Zero(cloud->size(), 3);
        color.setConstant(-1);
        colors.push_back(color);

        MatrixXfRowMajor row_major;
        H5::H5File transform_h5f(transform_files[k], H5F_ACC_RDONLY);
        load_hdf_dataset(transform_h5f, "transform", row_major, H5::PredType::NATIVE_FLOAT);
        transform_h5f.close();
        Eigen::MatrixXf transform(row_major);
        transforms.push_back(transform);

        ++show_progress;
    }

    initialized = true;
}

CloudServer& server()
{
    return CloudServer::Instance();
}


void CloudServer::saveColor(int ind)
{
    MatrixXiRowMajor row_major(colors[ind]);
    H5::H5File color_h5f(color_files[ind], H5F_ACC_TRUNC);
    write_hdf_dataset(color_h5f, "/color", row_major, H5::PredType::NATIVE_INT);
}
