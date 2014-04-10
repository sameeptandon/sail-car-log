#include <string>

#include <boost/progress.hpp>

#include <pcl/common/transforms.h>

#include "parameters.h"
#include "cloud_server.h"
#include "../videoreader/VideoReader.h"
#include "utils/cv_utils.h"

int main(int argc, char** arv)
{
    params().initialize();
    int start = params().start;
    int count = params().count;
    int step = params().step;
    int end = params().end;

    // Initialize cloud server

    server().initialize(params().pcd_dir, params().color_dir,
            params().h5_dir, params().count);

    // Initialize reader

    VideoReader reader(params().dset_dir, params().dset_avi);
    reader.setFrame(start);
    cv::Mat frame;

    std::vector<cv::Point2f> pixels;
    std::vector<cv::Point2f> filtered_pixels;
    std::vector<cv::Vec3b> pixel_colors;

    boost::progress_display show_progress(count);
    for (int k = start; k < end; k += step)
    {
        int num_steps = (k - start) / step;

        // Read frame

        bool success = reader.getNextFrame(frame);
        if (!success)
        {
            std::cout << "Reached end of video before expected" << std::endl;
            return 1;
        }

        // TODO Change this here to return point clouds over window
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = server().clouds[num_steps];

        Eigen::MatrixXi& color = server().colors[num_steps];
        Eigen::Matrix4f transform = server().transforms[num_steps];

        // Transform point cloud
        // We start with the clouds wrt imu_0

        // Transform to imu_t
        pcl::transformPointCloud(*cloud, *cloud, transform.inverse().eval());
        // Transform to lidar_t
        pcl::transformPointCloud(*cloud, *cloud, params().T_from_i_to_l);
        // Transform to camera_t
        pcl::transformPointCloud(*cloud, *cloud, params().trans_from_l_to_c);
        pcl::transformPointCloud(*cloud, *cloud, params().rot_to_c_from_l);

        // Filter point cloud

        pcl::PointCloud<PointXYZ>::Ptr final_cloud(new pcl::PointCloud<PointXYZ>());
        std::vector<int> filtered_indices;
        filter_lidar_cloud(cloud, final_cloud, filtered_indices, params().lidar_project_min_dist);
        assert (final_cloud->size() == filtered_indices.size());

        // Get colors by projecting the point cloud

        project_cloud_eigen(final_cloud, Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(),
                params().intrinsics, params().distortions, pixels);

        std::cout << "projected point pixels: " << pixels.size() << std::endl;

        // Filter pixels

        std::vector<int> filtered_indices_indices;
        filter_pixels(pixels, frame, filtered_pixels, filtered_indices_indices);
        std::cout << "filtered pixels: " << filtered_pixels.size() << std::endl;
        std::cout << "filtered_filtered_indices: " << filtered_indices_indices.size() << std::endl;

        std::vector<int> final_indices;
        BOOST_FOREACH(int ind, filtered_indices_indices)
        {
            final_indices.push_back(filtered_indices[ind]);
        }

        // Finally set some colors

        get_pixel_colors(filtered_pixels, frame, pixel_colors);
        assert (pixel_colors.size() == final_indices.size());
        std::cout << "coloring " << pixel_colors.size() << " points" << std::endl;
        for (int j = 0; j < pixel_colors.size(); j++)
        {
            // cv gives bgr so reverse
            color(final_indices[j], 0) = pixel_colors[j][2];
            color(final_indices[j], 1) = pixel_colors[j][1];
            color(final_indices[j], 2) = pixel_colors[j][0];
        }

        server().saveColor(num_steps);

        // Skip

        success = reader.skip(step - 1);
        if (!success)
        {
            std::cout << "Reached end of video before expected" << std::endl;
            return 1;
        }

        ++show_progress;
    }

    return 0;
}
