#include <string>
#include <Eigen/Dense>

// Just reads and writes binary files containing matrices of LIDAR data

void
readLDRFile(const std::string& file, Eigen::MatrixXf& data);

void
writeLDRFile(const std::string& file, const Eigen::MatrixXf& data);
