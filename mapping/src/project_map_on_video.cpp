#include <string>
#include <limits>

#include <boost/progress.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "videoreader/VideoReader.h"
#include "utils/cloud_utils.h"
#include "utils/hdf_utils.h"
#include "utils/cv_utils.h"
#include "utils/path_utils.h"
#include "parameters.h"

// TODO Speed things up...by a lot
//      one thing to start w/ is taking small chunk of map
// TODO Color pixels in-between
// TODO Use a z-buffer for the images

//namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
    std::string pcd_file = argv[1];

    params().initialize();
    int start = params().start;
    int step = params().step;
    int end = params().end;

    // Initialize reader

    VideoReader reader(params().dset_dir, params().dset_avi);
    reader.setFrame(start-1);

    // Set up a writer as well

    cv::VideoWriter writer;
    int fps = 30;
    //writer.open(argv[1], reader.getCodecType(), fps, cv::Size(reader.getFrameWidth(), reader.getFrameHeight()), true);
    if (argc > 1)
    {
        writer.open(argv[2], CV_FOURCC( 'M', 'J', 'P', 'G' ), fps, cv::Size(reader.getFrameWidth(), reader.getFrameHeight()), true);
        if (!writer.isOpened())
            std::cout << "Could not open " << argv[1] << " to write video" << std::endl;
    }

    cv::namedWindow("video", CV_WINDOW_AUTOSIZE);

    // Some data structures we'll use frequently later

    cv::Mat frame;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::vector<int> filtered_indices;
    std::vector<cv::Point2f> pixels;
    std::vector<cv::Point2f> filtered_pixels;

    MatrixXfRowMajor row_major;

    // Load cloud

    std::cout << "Loading point cloud..." << std::endl;
    load_cloud(pcd_file, cloud);
    std::cout << "Done." << std::endl;

    // In case point cloud is really big, can just cut it short and
    // save part of it here

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
    //filter_cloud(cloud, cloud_out, filtered_indices, "x", -500, 0);  // PARAM
    //pcl::io::savePCDFileBinary("tmp.pcd", *cloud_out);

    for (int k = start; k < end; k += step)
    {
        int num_steps = (k - start) / step;

        std::cout << num_steps << "/" << end / step << std::endl;

        std::string transform_file = (fs::path(params().h5_dir) / (boost::format("%1%.transform") % num_steps).str()).string();

        // Load transform from imu_0 to imu_t

        H5::H5File transform_h5f(transform_file, H5F_ACC_RDONLY);
        load_hdf_dataset(transform_h5f, "transform", row_major, H5::PredType::NATIVE_FLOAT);
        transform_h5f.close();
        Eigen::Matrix4f transform(row_major);

        // Read frame

        bool success = false;
        for (int j = 0; j < step; j++)
            success = reader.getNextFrame(frame);

        if (!success)
        {
            std::cout << "Reached end of video before expected" << std::endl;
            return 1;
        }

        // Transform point cloud
        // We start with the clouds wrt imu_0

        std::cout << "Transforming point cloud" << std::endl;

        // Transform to imu_t
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform.inverse().eval());

        // Filter point cloud

        std::cout << "Filtering point cloud" << std::endl;

        // PARAM
        filter_cloud(transformed_cloud, cloud_filtered, filtered_indices, "x", 0, 100);

        // Transform smaller filtered point cloud

        std::cout << "Transforming point cloud some more" << std::endl;

        // Transform to lidar_t
        pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, params().T_from_i_to_l);
        // Transform to camera_t
        pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, params().trans_from_l_to_c);
        pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, params().rot_to_c_from_l);

        // Get portion of point cloud that's within certain distance in front
        // of the camera

        std::cout << "Projecting cloud" << std::endl;

        // Project point cloud

        project_cloud_eigen(cloud_filtered, Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(),
                params().intrinsics, params().distortions, pixels);

        // Change the pixel colors

        std::cout << "Updating pixel colors" << std::endl;

        std::vector<int> filtered_pixel_indices;
        filter_pixels(pixels, frame, filtered_pixels, filtered_pixel_indices);

        // TODO Change pixel values in the image

        BOOST_FOREACH(int pixel_ind, filtered_pixel_indices)
        {
            cv::Point2f px = pixels[pixel_ind];
            pcl::PointXYZRGB pt = cloud_filtered->at(pixel_ind);
            frame.at<cv::Vec3b>(px.y, px.x) = cv::Vec3b(pt.b, pt.g, pt.r);
        }

        cv::imshow("video", frame);
        int key = cv::waitKey(1);
        if (key == 113)
            break;

        if (writer.isOpened())
            writer.write(frame);

        cv::imwrite((boost::format("%1%.jpg") % num_steps).str(), frame);

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
