#include <cstdio>
#include <fstream>
#include <iostream>
#include <vector>
#include "utils/ldr_utils.h"


void
readLDRFile(const std::string& file, Eigen::MatrixXf& data)
{
    std::ifstream in(file.c_str(), std::ios::in | std::ios::binary);
    in.seekg(0, std::ios::end);
    int size = in.tellg();
    in.seekg(0, std::ios::beg);
    
    int num_floats = size / (sizeof(float) / sizeof (char));
    int num_rows = num_floats / 6;
    data.resize(6, num_rows);

    float* row_arr = new float[num_floats];
    in.read((char*)(row_arr), size);

    float* data_arr = data.data();
    for (int k = 0; k < num_floats; k++)
        data_arr[k] = row_arr[k];

    data.transposeInPlace();

    in.close();
}


void
writeLDRFile(const std::string& file, const Eigen::MatrixXf& data)
{
    FILE *ldrFile = fopen(file.c_str(), "wb");

    for (int r = 0; r < data.rows(); r++)
    {
        Eigen::VectorXf v = data.row(r);
        float pBuffer[] = {v[0], v[1], v[2], v[3], v[4], v[5]};
        fwrite(pBuffer, 1, sizeof(pBuffer), ldrFile);
    }

    fclose(ldrFile);
};
