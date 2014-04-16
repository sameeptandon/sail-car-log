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
    std::string pcd_downsampled_dir;
    std::string params_file;
    std::string color_dir;
    std::string color_clouds_dir;

    int start;
    int step;
    int count;
    int end;

    int cam_ind;
    float lidar_project_min_dist;

    int map_color_window;
    int cloud_max_store;

    bool handle_occlusions;
    float octree_res;
    float color_octree_res;
    float prob_hit;
    float prob_miss;
    float occupancy_thres;
    float clamping_thres_max;
    float clamping_thres_min;
    float raycast_tol;
    bool cast_once;

    std::string octomap_file;
    std::string centered_octomap_file;
    std::string color_octomap_file;
    std::string centered_color_octomap_file;
    std::string octomap_h5_file;
    std::string color_octomap_h5_file;

    // Calibration parameters

    Eigen::Matrix4f T_from_i_to_l;
    Eigen::Matrix4f trans_from_i_to_l;
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
