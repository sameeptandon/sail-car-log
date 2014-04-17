#include <boost/progress.hpp>
#include "cloud_server.h"
#include "utils/path_utils.h"


void CloudServer::initialize(const std::string& cloud_dir, const std::string& color_dir, const std::string& transforms_dir, int size_window, int store_max)
{
    if (initialized)
        return;

    cloud_path = cloud_dir;
    color_path = color_dir;
    transforms_path = transforms_dir;
    window_size = size_window;
    max_store = store_max;

    // FIXME Assumes have at least max_store files and k=0; k++ indexed files
    get_range_files(cloud_path, 0, 1, max_store, "%1%.pcd", cloud_files);
    get_range_files(color_path, 0, 1, max_store, "%1%.h5", color_files);
    get_range_files(transforms_path, 0, 1, max_store, "%1%.transform", transform_files);
    assert (cloud_files.size() == color_files.size());
    assert (color_files.size() == transform_files.size());

    std::cout << "Initializing server..." << std::endl;
    forward(0);
    std::cout << "Done initializing server" << std::endl;
    initialized = true;
    start_ind = 0;
}

void CloudServer::forward(int ind_start)
{
    start_ind = ind_start;

    clouds.clear();
    colors.clear();
    transforms.clear();

    boost::progress_display show_progress(max_store);

    for (int k = start_ind; k < start_ind + max_store; k++)
    {
        if (k >= cloud_files.size())
            break;

        pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>());
        load_cloud(cloud_files[k], cloud);
        clouds.push_back(cloud);

        MatrixXiRowMajor row_major_i;
        MatrixXfRowMajor row_major_f;

        Eigen::MatrixXi color;
        if (boost::filesystem::exists(color_files[k]))
        {
            H5::H5File color_h5f(color_files[k], H5F_ACC_RDONLY);
            load_hdf_dataset(color_h5f, "color", row_major_i, H5::PredType::NATIVE_INT);
            color = Eigen::MatrixXi(row_major_i);
        }
        else
        {
            color = Eigen::MatrixXi::Zero(cloud->size(), 3);
            color.setConstant(-1);
        }
        colors.push_back(color);

        H5::H5File transform_h5f(transform_files[k], H5F_ACC_RDONLY);
        load_hdf_dataset(transform_h5f, "transform", row_major_f, H5::PredType::NATIVE_FLOAT);
        transform_h5f.close();
        Eigen::MatrixXf transform(row_major_f);
        transforms.push_back(transform);

        ++show_progress;
    }
}

void CloudServer::getCurrentWindow(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds_in_window, std::vector<Eigen::MatrixXi*>& colors_in_window)
{
    clouds_in_window.clear();
    colors_in_window.clear();

    // FIXME
    for (int k = start_ind; k < start_ind + max_store; k++)
    {
        if (k >= colors.size())
            break;
        clouds_in_window.push_back(clouds[k]);
        colors_in_window.push_back(&colors[k]);
    }
}


void CloudServer::saveColor(int ind)
{
    MatrixXiRowMajor row_major(colors[ind]);
    H5::H5File color_h5f(color_files[ind], H5F_ACC_TRUNC);
    write_hdf_dataset(color_h5f, "/color", row_major, H5::PredType::NATIVE_INT);
    color_h5f.close();
}


void CloudServer::saveCurrentColorWindow()
{
    // FIXME
    for (int k = start_ind; k < start_ind + max_store; k++)
    {
        if (k >= colors.size())
            break;
        saveColor(k);
    }
}


CloudServer& server()
{
    return CloudServer::Instance();
}
