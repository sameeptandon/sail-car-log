#include "parameters.h"
#include <boost/python.hpp>
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
   params_file = py::extract<std::string>(pycfg.attr("PARAMS_H5_FILE"));
   cam_ind = py::extract<int>(pycfg.attr("CAM_NUM")) - 1;
   lidar_project_min_dist = py::extract<float>(pycfg.attr("LIDAR_PROJECT_MIN_DIST"));

   // Loading calibration parameters

    H5::H5File params_h5f(params_file, H5F_ACC_RDONLY);
    MatrixXfRowMajor row_major;

    load_hdf_dataset(params_h5f, "/lidar/T_from_l_to_i", row_major, H5::PredType::NATIVE_FLOAT);
    Eigen::Matrix4f T_from_l_to_i(row_major);
    T_from_i_to_l = T_from_l_to_i.inverse();

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
