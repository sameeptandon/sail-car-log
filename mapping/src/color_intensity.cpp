#include <iostream>
#include "utils/cloud_utils.h"
#include "utils/color_utils.h"

int main(int argc, char** argv)
{

    if (argc < 3)
    {
        std::cout << "Usage: ./color_cloud [pcd_file] [output_pcd]" << std::endl;
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    load_cloud(argv[1], cloud);

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
        cv::Vec3b col = rgb.at<cv::Vec3b>(k, 0);
        rgb_pt.r = col(0);
        rgb_pt.g = col(1);
        rgb_pt.b = col(2);

        rgb_cloud->push_back(rgb_pt);
    }

    pcl::io::savePCDFileBinary(argv[2], *rgb_cloud);

    return 0;
}
