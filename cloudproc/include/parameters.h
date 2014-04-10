#pragma once

#include <string>
#include "utils/hdf_utils.h"

class Parameters
{
  public:
    static Parameters& Instance()
    {
        static Parameters instance;
        return instance;
    }

    bool initialized;
    void initialize();

    // General settings / config

    std::string dset_dir;
    std::string dset_avi;
    std::string h5_dir;
    std::string pcd_dir;
    std::string params_file;

    int start;
    int step;
    int count;
    int end;

    int cam_ind;
    float lidar_project_min_dist;

    // Calibration parameters

    Eigen::Matrix4f T_from_i_to_l;
    Eigen::Matrix4f trans_from_l_to_c;
    Eigen::Matrix4f rot_to_c_from_l;
    Eigen::Matrix3f intrinsics;
    Eigen::VectorXf distortions;

  private:
    Parameters() { initialized = false; };
    Parameters(Parameters const&);
    void operator=(Parameters const&);
};

Parameters& params();
