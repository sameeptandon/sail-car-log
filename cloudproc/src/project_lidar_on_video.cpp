#include <string>
#include <limits>

#include <boost/progress.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include "parameters.h"
#include "videoreader/VideoReader.h"
#include "utils/cloud_utils.h"
#include "utils/hdf_utils.h"
#include "utils/cv_utils.h"

//namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
    params().initialize();
    int start = params().start;
    int step = params().step;
    int end = params().end;

    // Initialize reader

    VideoReader reader(params().dset_dir, params().dset_avi);
    reader.setFrame(start);

    cv::namedWindow("video", CV_WINDOW_AUTOSIZE);
    cv::Mat frame;

    pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>());
    std::vector<cv::Point2f> pixels;
    std::vector<cv::Point2f> filtered_pixels;

    MatrixXfRowMajor row_major;

    for (int k = start; k < end; k += step)
    {
        int num_steps = (k - start) / step;

        std::string pcd_file = (fs::path(params().pcd_dir) / (boost::format("%1%.pcd") % num_steps).str()).string();
        std::string transform_file = (fs::path(params().h5_dir) / (boost::format("%1%.transform") % num_steps).str()).string();

        // Load cloud

        load_cloud(pcd_file, cloud);

        // Load transform from imu_0 to imu_t

        H5::H5File transform_h5f(transform_file, H5F_ACC_RDONLY);
        load_hdf_dataset(transform_h5f, "transform", row_major, H5::PredType::NATIVE_FLOAT);
        transform_h5f.close();
        Eigen::Matrix4f transform(row_major);

        // Read frame
        // TODO Project on at many different time steps

        bool success = reader.getNextFrame(frame);
        if (!success)
        {
            std::cout << "Reached end of video before expected" << std::endl;
            return 1;
        }

        // Transform point cloud
        // We start with the clouds wrt imu_0

        // Transform to imu_t
        transform = transform.inverse().eval();
        pcl::transformPointCloud(*cloud, *cloud, transform);
        // Transform to lidar_t
        pcl::transformPointCloud(*cloud, *cloud, params().T_from_i_to_l);
        // Transform to camera_t
        pcl::transformPointCloud(*cloud, *cloud, params().trans_from_l_to_c);
        pcl::transformPointCloud(*cloud, *cloud, params().rot_to_c_from_l);

        // Filter point cloud

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0, std::numeric_limits<float>::max());
        pass.filter(*cloud);

        pcl::PointCloud<PointXYZ>::Ptr final_cloud(new pcl::PointCloud<PointXYZ>());
        BOOST_FOREACH(pcl::PointXYZ p, cloud->points)
        {
            if (p.x*p.x + p.y*p.y + p.z*p.z > params().lidar_project_min_dist)
                final_cloud->push_back(p);
        }

        // Project point cloud

        project_cloud_eigen(final_cloud, Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(),
                params().intrinsics, params().distortions, pixels);

        // Change the pixel colors

        filter_pixels(pixels, frame, filtered_pixels);
        set_pixel_colors(filtered_pixels, cv::Vec3b(0, 0, 255), frame, 4);

        // Show

        cv::imshow("video", frame);
        int key = cv::waitKey(1);
        if (key == 113)
            break;

        // Skip

        success = reader.skip(step - 1);
        if (!success)
        {
            std::cout << "Reached end of video before expected" << std::endl;
            return 1;
        }
    }

    return 0;
}
