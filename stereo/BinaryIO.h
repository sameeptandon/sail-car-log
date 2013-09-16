#ifndef BINARY_IO_H
#define BINARY_IO_H

const static int32_t FORMAT_UINT8 = 1;
const static int32_t FORMAT_SHORT = 2;
const static int32_t FORMAT_FLOAT = 4;
const static int32_t FORMAT_DOUBLE = 8;

int prod(const std::vector<int>& dims) {
  int total = 1;
  for (int i = 0; i < dims.size(); i++) total *= dims[i];
  return total;
}

template <class U, class T>
void writeData(std::ostream& ofs, const std::vector<T>& data) {
  static const int BUF_LEN = 2048;
  U buffer[BUF_LEN];

  int i;
  for (i = 0; i+BUF_LEN <= data.size(); i += BUF_LEN) {
    std::copy(data.begin() + i, data.begin() + i + BUF_LEN, buffer); // convert
    ofs.write((const char*)buffer, sizeof(U) * BUF_LEN); // write block
  }
  if (data.size() > i) {
    std::copy(data.begin() + i, data.end(), buffer); // convert
    ofs.write((const char*)buffer, sizeof(U) * (data.size()-i)); // write remaining
  }
  assert(!ofs.fail());
}


template <class U, class T>
void readData(std::istream& ifs, std::vector<T>* data) {
  static const int BUF_LEN = 2048;
  U buffer[BUF_LEN];
  int i;
  for (i = 0; i+BUF_LEN <= data->size(); i += BUF_LEN) {
    ifs.read((char*)buffer, sizeof(U) * BUF_LEN); // read block
    std::copy(buffer, buffer+BUF_LEN, data->begin() + i); // convert
  }
  if (data->size() > i) {
    ifs.read((char*)buffer, sizeof(U) * (data->size()-i)); // read remaining
    std::copy(buffer, buffer + data->size() - i, data->begin() + i); // convert
  }
  assert(!ifs.fail());
}

template <class T>
void writeBin(const char* fname, const std::vector<T>& data, const std::vector<int>& dims,
	      int outputFormat) {
  std::ofstream ofs(fname, std::ios::binary);
  int N = dims.size();
  int32_t type = outputFormat;
  ofs.write((const char*)&type, sizeof(int32_t));
  const int n = N;
  ofs.write((const char*)&n, sizeof(int32_t));
  assert(sizeof(int) == sizeof(int32_t));
  ofs.write((const char*)dims.data(), sizeof(int32_t)*N);
  if (ofs.fail()) {
    std::cout << "Failed to output to " << fname << std::endl;
    assert(!ofs.fail());
  }
  
  // write data
  switch (outputFormat) {
  case FORMAT_UINT8:
    writeData<uint8_t,T>(ofs, data);
    break;
  case FORMAT_SHORT:
    writeData<short,T>(ofs, data);
    break;
  case FORMAT_FLOAT:
    writeData<float,T>(ofs, data);
    break;
  case FORMAT_DOUBLE:
    writeData<double,T>(ofs, data);
    break;
  }
}

template <class T>
void readBin(const char* fname, std::vector<T>* data, std::vector<int>* dims) {
  std::ifstream ifs(fname, std::ios::binary);
  int N = dims->size();
  int32_t type;
  ifs.read((char*)&type, sizeof(int32_t));
  int n = -1;
  ifs.read((char*)&n, sizeof(int32_t));
  assert(n == N);
  ifs.read((char*)dims->data(), sizeof(int32_t)*N);
  assert(!ifs.fail());
  
  int length = prod(*dims);
  data->resize(length);

  // read data
  switch (type) {
  case FORMAT_UINT8:
    readData<uint8_t,T>(ifs, data);
    break;
  case FORMAT_SHORT:
    writeData<short,T>(ifs, data);
    break;    
  case FORMAT_FLOAT:
    readData<float,T>(ifs, data);
    break;
  case FORMAT_DOUBLE:
    readData<double,T>(ifs, data);
    break;
  }
}


#endif
