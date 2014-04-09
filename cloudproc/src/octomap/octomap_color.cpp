#include <string>
#include <limits>

#include <boost/progress.hpp>
#include <boost/python.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include "../videoreader/VideoReader.h"
#include "utils/cloud_utils.h"
#include "utils/hdf_utils.h"
#include "utils/cv_utils.h"

//namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace py = boost::python;

int main(int argc, char** argv)
{
    Py_Initialize();

    // Too many options to pass in, just import from python file...
    py::object pycfg = py::import("pipeline_config");
    std::string dset_dir = py::extract<std::string>(pycfg.attr("DSET_DIR"));
    std::string dset_avi = py::extract<std::string>(pycfg.attr("DSET_AVI"));
    int start = py::extract<int>(pycfg.attr("EXPORT_START"));
    int step = py::extract<int>(pycfg.attr("EXPORT_STEP"));
    int count = py::extract<int>(pycfg.attr("EXPORT_NUM"));
    int end = start + step * count;
    std::string h5_dir = py::extract<std::string>(pycfg.attr("POINTS_H5_DIR"));
    std::string pcd_dir = py::extract<std::string>(pycfg.attr("PCD_DIR"));
    std::string params_file = py::extract<std::string>(pycfg.attr("PARAMS_H5_FILE"));
    int cam_ind = py::extract<int>(pycfg.attr("CAM_NUM")) - 1;
    int lidar_project_min_dist = py::extract<float>(pycfg.attr("LIDAR_PROJECT_MIN_DIST"));

    // Read in params and set up transforms

    H5::H5File params_h5f(params_file, H5F_ACC_RDONLY);

    MatrixXfRowMajor row_major;
    load_hdf_dataset(params_h5f, "/lidar/T_from_l_to_i", row_major, H5::PredType::NATIVE_FLOAT);
    Eigen::Matrix4f T_from_l_to_i(row_major);
    Eigen::Matrix4f T_from_i_to_l = T_from_l_to_i.inverse();

    Eigen::Matrix4f trans_from_l_to_c = Eigen::Matrix4f::Identity();
    std::string s = (boost::format("/cam/%1%/displacement_from_l_to_c_in_lidar_frame") % cam_ind).str();
    load_hdf_dataset(params_h5f, s, row_major, H5::PredType::NATIVE_FLOAT);
    Eigen::Vector3f displacement_from_l_to_c_in_lidar_frame(row_major);
    trans_from_l_to_c.block(0, 3, 3, 1) = displacement_from_l_to_c_in_lidar_frame;

    Eigen::Matrix4f rot_to_c_from_l = Eigen::Matrix4f::Identity();
    s = (boost::format("/cam/%1%/R_to_c_from_l_in_camera_frame") % cam_ind).str();
    load_hdf_dataset(params_h5f, s, row_major, H5::PredType::NATIVE_FLOAT);
    Eigen::Matrix3f R_to_c_from_l_in_camera_frame(row_major);
    // FIXME Too much hard-coding
    Eigen::Matrix3f swap;
    swap << 0.0, -1.0, 0.0,
            0.0, 0.0, -1.0,
            1.0, 0.0, 0.0;
    rot_to_c_from_l.block(0, 0, 3, 3) = R_to_c_from_l_in_camera_frame * swap;

    s = (boost::format("/cam/%1%/KK") % cam_ind).str();
    load_hdf_dataset(params_h5f, s, row_major, H5::PredType::NATIVE_FLOAT);
    Eigen::Matrix3f intrinsics(row_major);

    s = (boost::format("/cam/%1%/distort") % cam_ind).str();
    load_hdf_dataset(params_h5f, s, row_major, H5::PredType::NATIVE_FLOAT);
    Eigen::VectorXf distortions(row_major);

    params_h5f.close();

    // TODO Use ICP transforms as well

    std::cout << "range [" << start << ", " << step << ", " << count << "]" << std::endl;

    // Initialize reader

    VideoReader reader(dset_dir, dset_avi);
    reader.setFrame(start);

    cv::namedWindow("video", CV_WINDOW_AUTOSIZE);
    cv::Mat frame;

    pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>());
    std::vector<cv::Point2f> pixels;
    std::vector<cv::Point2f> filtered_pixels;

    for (int k = start; k < end; k += step)
    {
        int num_steps = (k - start) / step;

        std::string pcd_file = (fs::path(pcd_dir) / (boost::format("%1%.pcd") % num_steps).str()).string();
        std::string transform_file = (fs::path(h5_dir) / (boost::format("%1%.transform") % num_steps).str()).string();

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
        pcl::transformPointCloud(*cloud, *cloud, T_from_i_to_l);
        // Transform to camera_t
        pcl::transformPointCloud(*cloud, *cloud, trans_from_l_to_c);
        pcl::transformPointCloud(*cloud, *cloud, rot_to_c_from_l);

        // Filter point cloud

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0, std::numeric_limits<float>::max());
        pass.filter(*cloud);

        pcl::PointCloud<PointXYZ>::Ptr final_cloud(new pcl::PointCloud<PointXYZ>());
        BOOST_FOREACH(pcl::PointXYZ p, cloud->points)
        {
            if (p.x*p.x + p.y*p.y + p.z*p.z > lidar_project_min_dist)
                final_cloud->push_back(p);
        }

        // Project point cloud

        project_cloud_eigen(final_cloud, Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(), intrinsics, distortions, pixels);

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
