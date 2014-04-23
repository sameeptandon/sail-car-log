#include <iostream>
#include "utils/cloud_utils.h"
#include "utils/hdf_utils.h"
#include "utils/color_utils.h"


int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: ./color_cloud [pcd_file] [color_h5_file] [output_pcd]" << std::endl;
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    load_cloud(argv[1], cloud);

    H5::H5File color_h5f(argv[2], H5F_ACC_RDONLY);
    MatrixXiRowMajor row_major;
    load_hdf_dataset(color_h5f, "/color", row_major, H5::PredType::NATIVE_INT);
    color_h5f.close();
    Eigen::MatrixXi color(row_major);

    assert (cloud->size() == color.rows());

    std::vector<float> intensities;
    cv::Mat rgb(cloud->size(), 1, CV_8UC3);
    for (int k = 0; k < cloud->size(); k++)
    {
        intensities.push_back((cloud->at(k)).intensity);
    }
    heatmap_color(intensities, rgb, 0, 100);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int k = 0; k < cloud->size(); k++)
    {
        pcl::PointXYZI pt = cloud->at(k);

        pcl::PointXYZRGB rgb_pt;
        rgb_pt.x = pt.x;
        rgb_pt.y = pt.y;
        rgb_pt.z = pt.z;
        if (color(k, 0) == -1)  // Never assigned a color
        {
            cv::Vec3b col = rgb.at<cv::Vec3b>(k, 0);
            rgb_pt.r = col(0);
            rgb_pt.g = col(1);
            rgb_pt.b = col(2);
        }
        else
        {
            rgb_pt.r = color(k, 0);
            rgb_pt.g = color(k, 1);
            rgb_pt.b = color(k, 2);
        }

        rgb_cloud->push_back(rgb_pt);
    }

    if (rgb_cloud->size() == 0)
    {
        std::cout << "empty cloud for " << argv[1] << ", skipping" << std::endl;
        return 0;
    }
    pcl::io::savePCDFileBinary(argv[3], *rgb_cloud);
    //pcl::io::savePCDFileASCII(argv[3], *rgb_cloud);

    return 0;
}
