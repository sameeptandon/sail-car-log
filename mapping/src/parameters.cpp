#include "parameters.h"
#include <boost/format.hpp>

namespace py = boost::python;

void Parameters::initialize()
{
    if (initialized)
        return;

   Py_Initialize();

   py::object pycfg = py::import("pipeline_config");

   // Importing parameters from config file

   dset_dir = py::extract<std::string>(pycfg.attr("DSET_DIR"));
   dset_avi = py::extract<std::string>(pycfg.attr("DSET_AVI"));
   start = py::extract<int>(pycfg.attr("EXPORT_START"));
   step = py::extract<int>(pycfg.attr("EXPORT_STEP"));
   count = py::extract<int>(pycfg.attr("EXPORT_NUM"));
   end = start + step * count;
   h5_dir = py::extract<std::string>(pycfg.attr("POINTS_H5_DIR"));
   pcd_dir = py::extract<std::string>(pycfg.attr("PCD_DIR"));
   pcd_downsampled_dir = py::extract<std::string>(pycfg.attr("PCD_DOWNSAMPLED_DIR"));
   color_dir = py::extract<std::string>(pycfg.attr("COLOR_DIR"));
   color_clouds_dir = py::extract<std::string>(pycfg.attr("COLOR_CLOUDS_DIR"));
   params_file = py::extract<std::string>(pycfg.attr("PARAMS_H5_FILE"));
   cam_ind = py::extract<int>(pycfg.attr("CAM_NUM")) - 1;
   lidar_project_min_dist = py::extract<float>(pycfg.attr("LIDAR_PROJECT_MIN_DIST"));
   py::list l = py::extract<py::list>(pycfg.attr("ICP_COORD_WEIGHTS"));
   for (int j = 0; j < py::len(l); j++)
       icp_coord_weights.push_back(py::extract<float>(l[j]));
   icp_tol = py::extract<float>(pycfg.attr("ICP_TOL"));
   icp_min_intensity = py::extract<float>(pycfg.attr("ICP_MIN_INTENSITY"));
   map_color_window = py::extract<int>(pycfg.attr("MAP_COLOR_WINDOW"));
   cloud_max_store = py::extract<int>(pycfg.attr("CLOUD_MAX_STORE"));
   handle_occlusions = py::extract<bool>(pycfg.attr("HANDLE_OCCLUSIONS"));
   octree_res = py::extract<float>(pycfg.attr("OCTOMAP_RES"));
   color_octree_res = py::extract<float>(pycfg.attr("COLOR_OCTOMAP_RES"));
   prob_hit = py::extract<float>(pycfg.attr("PROB_HIT"));
   prob_miss = py::extract<float>(pycfg.attr("PROB_MISS"));
   occupancy_thres = py::extract<float>(pycfg.attr("OCCUPANCY_THRES"));
   clamping_thres_max = py::extract<float>(pycfg.attr("CLAMPING_THRES_MAX"));
   clamping_thres_min = py::extract<float>(pycfg.attr("CLAMPING_THRES_MIN"));
   raycast_tol = py::extract<float>(pycfg.attr("RAYCAST_TOL"));
    cast_octomap_single = py::extract<bool>(pycfg.attr("CAST_OCTOMAP_SINGLE"));
   cast_once = py::extract<bool>(pycfg.attr("CAST_ONCE"));
   octomap_file = py::extract<std::string>(pycfg.attr("OCTOMAP_FILE"));
   centered_octomap_file = py::extract<std::string>(pycfg.attr("CENTERED_OCTOMAP_FILE"));
   color_octomap_file = py::extract<std::string>(pycfg.attr("COLOR_OCTOMAP_FILE"));
   centered_color_octomap_file = py::extract<std::string>(pycfg.attr("CENTERED_COLOR_OCTOMAP_FILE"));
   octomap_h5_file = py::extract<std::string>(pycfg.attr("OCTOMAP_H5_FILE"));
   color_octomap_h5_file = py::extract<std::string>(pycfg.attr("COLOR_OCTOMAP_H5_FILE"));

    pyListToVector(pycfg.attr("OCTOMAP_SINGLE_FILES"), octomap_single_files);

   // Loading calibration parameters

    H5::H5File params_h5f(params_file, H5F_ACC_RDONLY);
    MatrixXfRowMajor row_major;

    load_hdf_dataset(params_h5f, "/lidar/T_from_l_to_i", row_major, H5::PredType::NATIVE_FLOAT);
    Eigen::Matrix4f T_from_l_to_i(row_major);
    T_from_i_to_l = T_from_l_to_i.inverse();

    trans_from_i_to_l = Eigen::Matrix4f::Identity();
    trans_from_i_to_l.block(0, 3, 3, 1) = T_from_i_to_l.block(0, 3, 3, 1);

    trans_from_l_to_c = Eigen::Matrix4f::Identity();
    std::string s = (boost::format("/cam/%1%/displacement_from_l_to_c_in_lidar_frame") % cam_ind).str();
    load_hdf_dataset(params_h5f, s, row_major, H5::PredType::NATIVE_FLOAT);
    Eigen::Vector3f displacement_from_l_to_c_in_lidar_frame(row_major);
    trans_from_l_to_c.block(0, 3, 3, 1) = displacement_from_l_to_c_in_lidar_frame;

    rot_to_c_from_l = Eigen::Matrix4f::Identity();
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
    Eigen::Matrix3f K(row_major);
    intrinsics = K;

    s = (boost::format("/cam/%1%/distort") % cam_ind).str();
    load_hdf_dataset(params_h5f, s, row_major, H5::PredType::NATIVE_FLOAT);
    Eigen::VectorXf D(row_major);
    distortions = D;

    params_h5f.close();

    initialized = true;
}

Parameters& params()
{
    return Parameters::Instance();
}
