#pragma once

#include <iostream>
#include "H5Cpp.h"

#include <Eigen/Dense>

// Reference: http://geodynamics.org/svn/cig/cs/cigma/trunk/src/io_hdf5.h

typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXiRowMajor;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfRowMajor;

template <typename T>
H5::DataType h5_datatype_from();

template <typename OutputMatrix> void
load_hdf_dataset(H5::H5File &file, std::string dataset_path, OutputMatrix &mat, H5::PredType type);

template <typename OutputMatrix> void
write_hdf_dataset(H5::H5File &file, std::string dataset_path, const OutputMatrix& mat, H5::PredType type);

template <typename AttrType> void
get_hdf_attribute(H5::H5File &file, std::string dataset_path, std::string attr_name, AttrType &attr);

template <typename AttrType> void
write_hdf_attribute(H5::H5File &file, std::string dataset_path, std::string attr_name, const AttrType& attr);

#include "hdf_utils.hpp"
