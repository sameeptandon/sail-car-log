#pragma once

#include "utils/hdf_utils.h"
#include "utils/cloud_utils.h"

class CloudServer
{
  public:
    static CloudServer& Instance()
    {
        static CloudServer instance;
        return instance;
    }

    void initialize(const std::string& cloud_dir, const std::string& color_dir, const std::string& transforms_dir, int store_max);
    // FIXME Can't go backward
    void forward(int start_ind);

    bool initialized;
    // Maximum number of clouds to store at one time
    int max_store;

    std::string cloud_path;
    std::string color_path;
    std::string transforms_path;

    std::vector<std::string> cloud_files;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

    // TODO Want to separate / integrate this differently
    std::vector<std::string> color_files;
    std::vector<Eigen::MatrixXi> colors;

    std::vector<std::string> transform_files;
    std::vector<Eigen::Matrix4f> transforms;

    void saveColor(int ind);

  private:
    CloudServer() { initialized = false; };
    CloudServer(CloudServer const&);
    void operator=(CloudServer const&);
};

CloudServer& server();
