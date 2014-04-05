template <typename OutputMatrix> void
load_hdf_dataset(H5::H5File &file, std::string dataset_path, OutputMatrix &mat, H5::PredType type)
{
  H5::DataSet dataset = file.openDataSet(dataset_path);
  H5::DataSpace dataspace = dataset.getSpace();
  //int rank = dataspace.getSimpleExtentNdims();
  //assert(rank == 2);

  hsize_t dims_out[2];
  dataspace.getSimpleExtentDims(dims_out, NULL);

  mat.resize(dims_out[0], dims_out[1]);
  dataset.read(mat.data(), type);
  dataset.close();
}


template <typename OutputMatrix> void
write_hdf_dataset(H5::H5File &file, std::string dataset_path, const OutputMatrix& mat, H5::PredType type)
{
  int rank = 2;
  hsize_t dims[] = {mat.rows(), mat.cols()};
  H5::DataSpace dataspace(rank, dims);
  H5::DataSet dataset(file.createDataSet(dataset_path, type, dataspace));
  dataset.write(mat.data(), type);
  dataset.close();
}


template <typename AttrType> void
get_hdf_attribute(H5::H5File &file, std::string dataset_path, std::string attr_name, AttrType &attr)
{
  H5::DataSet dataset = file.openDataSet(dataset_path);
  H5::Attribute attribute = dataset.openAttribute(attr_name);

  H5T_class_t typeclass = attribute.getTypeClass();
  H5::DataType datatype = attribute.getDataType();

  if (typeclass == H5T_STRING && !datatype.isVariableStr())
  {
    size_t size = datatype.getSize();
    char* buf = new char[size + 1];
    attribute.read(datatype, buf);
    buf[size] = '\0';
    attr = std::string(buf);
    delete[] buf;
  }
  else
  {
    attribute.read(h5_datatype_from<AttrType>(), attr);
  }

  attribute.close();
}

template <typename AttrType> void
write_hdf_attribute(H5::H5File &file, std::string dataset_path, std::string attr_name, const AttrType& attr)
{
  H5::DataSet dataset = file.openDataSet(dataset_path);
  // Currently only scalars
  H5::Attribute attribute(dataset.createAttribute(attr_name, h5_datatype_from<AttrType>(), H5::DataSpace()));

  H5T_class_t typeclass = attribute.getTypeClass();
  H5::DataType datatype = attribute.getDataType();

  attribute.write(h5_datatype_from<AttrType>(), attr);

  attribute.close();
}
