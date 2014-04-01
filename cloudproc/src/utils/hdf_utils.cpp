#include "odp/utils/hdf_utils.h"

template <> H5::DataType h5_datatype_from<int>()         { return H5::PredType::NATIVE_INT; }
template <> H5::DataType h5_datatype_from<long>()        { return H5::PredType::NATIVE_LONG; }
template <> H5::DataType h5_datatype_from<float>()       { return H5::PredType::NATIVE_FLOAT; }
template <> H5::DataType h5_datatype_from<double>()      { return H5::PredType::NATIVE_DOUBLE; }
template <> H5::DataType h5_datatype_from<std::string>() { return H5::StrType(0, H5T_STRING); }
