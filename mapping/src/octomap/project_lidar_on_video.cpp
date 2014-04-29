#include <string>
#include <limits>

#include <boost/progress.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <pcl/common/transforms.h>

#include <octomap/octomap.h>

#include "../videoreader/VideoReader.h"
#include "utils/cloud_utils.h"
#include "utils/hdf_utils.h"
#include "utils/cv_utils.h"
#include "utils/path_utils.h"
#include "parameters.h"

//namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
    params().initialize();
    int start = params().start;
    int step = params().step;
    int end = params().end;

    // Load octomap

    boost::shared_ptr<octomap::OcTree> octree((octomap::OcTree*)octomap::OcTree::read(params().octomap_file));
    std::cout << "Loaded octree of size " << octree->size() << std::endl;

    // Initialize reader

    VideoReader reader(params().dset_dir, params().dset_avi);
    reader.setFrame(start-1);

    // Set up a writer as well

    cv::VideoWriter writer;
    int fps = 30;
    //writer.open(argv[1], reader.getCodecType(), fps, cv::Size(reader.getFrameWidth(), reader.getFrameHeight()), true);
    if (argc > 1)
    {
        writer.open(argv[1], CV_FOURCC( 'M', 'J', 'P', 'G' ), fps, cv::Size(reader.getFrameWidth(), reader.getFrameHeight()), true);
        if (!writer.isOpened())
            std::cout << "Could not open " << argv[1] << " to write video" << std::endl;
     }

    cv::namedWindow("video", CV_WINDOW_AUTOSIZE);
    cv::Mat frame;

    pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>());
    pcl::PointCloud<PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<PointXYZ>());
    std::vector<cv::Point2f> pixels;
    std::vector<cv::Point2f> filtered_pixels;

    MatrixXfRowMajor row_major;

    for (int k = start; k < end; k += step)
    {
        int num_steps = (k - start) / step;

        //std::string pcd_file = (fs::path(params().pcd_dir) / (boost::format("%1%.pcd") % num_steps).str()).string();
        std::vector<std::string> pcd_files;
        get_range_files(params().pcd_dir, num_steps, 1, std::min(5, params().count - num_steps), "%1%.pcd", pcd_files);
        std::string transform_file = (fs::path(params().h5_dir) / (boost::format("%1%.transform") % num_steps).str()).string();

        // Load cloud

        //load_clouds(pcd_files, cloud);
        load_cloud(pcd_files[0], cloud);
        pcl::copyPointCloud(*cloud, *cloud_copy);

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
        pcl::transformPointCloud(*cloud, *cloud, transform.inverse().eval());
        // Transform to lidar_t
        pcl::transformPointCloud(*cloud, *cloud, params().T_from_i_to_l);
        // Transform to camera_t
        pcl::transformPointCloud(*cloud, *cloud, params().trans_from_l_to_c);
        pcl::transformPointCloud(*cloud, *cloud, params().rot_to_c_from_l);

        // Filter point cloud

        pcl::PointCloud<PointXYZ>::Ptr final_cloud(new pcl::PointCloud<PointXYZ>());
        std::vector<int> filtered_indices;
        filter_lidar_cloud(cloud, final_cloud, filtered_indices);

        // Project point cloud

        project_cloud_eigen(final_cloud, Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(),
                params().intrinsics, params().distortions, pixels);

        // Change the pixel colors

        std::vector<int> filtered_pixel_indices;
        filter_pixels(pixels, frame, filtered_pixels, filtered_pixel_indices);

        // Split pixels into those in the octomap and those not
        // Ideally those in the octomap are static objects and those not are dynamic

        std::vector<cv::Point2f> static_pixels;
        std::vector<cv::Point2f> dynamic_pixels;

        for (int j = 0; j < filtered_pixels.size(); j++)
        {
            int ind = filtered_indices[filtered_pixel_indices[j]];
            pcl::PointXYZ pt = cloud_copy->at(ind);
            octomap::OcTreeNode* node = octree->search(pt.x, pt.y, pt.z);
            octomap::OcTreeNode* node_below = octree->search(pt.x, pt.y + params().octree_res, pt.z);
            // FIXME PARAM
            if ((node && octree->isNodeOccupied(node)) || (node_below && octree->isNodeOccupied(node_below)) || cloud->at(ind).y < -3.0)
                static_pixels.push_back(filtered_pixels[j]);
            else
                dynamic_pixels.push_back(filtered_pixels[j]);
        }

        //set_pixel_colors(filtered_pixels, cv::Vec3b(0, 0, 255), frame, 4);
        set_pixel_colors(static_pixels, cv::Vec3b(0, 255, 0), frame, 4);
        set_pixel_colors(dynamic_pixels, cv::Vec3b(0, 0, 255), frame, 4);

        // Show

        cv::imshow("video", frame);
        int key = cv::waitKey(1);
        if (key == 113)
            break;

        if (writer.isOpened())
            writer.write(frame);

        // Skip

        success = reader.skip(step - 1);
        if (!success)
        {
            std::cout << "Reached end of video before expected" << std::endl;
            return 1;
        }
    }

    writer.release();

    return 0;
}
