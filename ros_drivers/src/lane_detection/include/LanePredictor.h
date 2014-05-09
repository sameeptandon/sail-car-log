#include "FastCpp.h"
//#include "gpu/kernels/GpuKernels.h"
#include <fstream>
#include <iostream>
#include "gpu/GpuBlockOps.h"
#include "gpu/GpuMath.h"
#include "io/ArrayIO.h"
#include "cuda_profiler_api.h"

class LanePredictor{

public:
	virtual Ptr<ArrayViewHandle> processImage(const Ptr<ArrayViewHandle>& img)=0;

protected:
	int stream;
	std::string model;
	Ptr<ArrayViewHandle> data_buf;
    Ptr<ArrayViewHandle> host_output;

	static unsigned int clp2(unsigned int x) {
		x = x - 1;
		x = x | (x >> 1);
		x = x | (x >> 2);
		x = x | (x >> 4);
		x = x | (x >> 8);
		x = x | (x >>16);
		return x + 1;
	}

	static void allocScratchArray(  const Ptr<ArrayViewHandle>& input,
	                         const Ptr<ArrayViewHandle>& output, 
	                         const Ptr<ArrayViewHandle>& filters,
	                         int stepy,
	                         Ptr<ArrayViewHandle>& inputScratch,
	                         Ptr<ArrayViewHandle>& outputScratch,
	                         int stream,
	                         int num_splits = 0){
    if(num_splits==0){
        num_splits = clp2((output->dim(2)*output->dim(3)-1)/65535+1);
    }
    int split_width = output->dim(3)/num_splits;
    if(split_width*num_splits != output->dim(3)){
        split_width += 1;
    }
    inputScratch =  gpuArrayAllocRM(input->dataType(),DDim(num_splits,input->dim(1),input->dim(2),filters->dim(5)+(split_width-1)*stepy),stream);
    outputScratch = gpuArrayAllocRM(output->dataType(),DDim(num_splits,output->dim(1),output->dim(2),split_width),stream);
};

	static void gpuFilterTimesLarge(  const Ptr<ArrayViewHandle>& filters,
	                                  bool transpose,
	                                  int stepz, int stepx, int stepy,
	                                  const Ptr<ArrayViewHandle>& input,
        	                          const Ptr<ArrayViewHandle>& output,
	                                  const Ptr<ArrayViewHandle>& inputScratch,
	                                  const Ptr<ArrayViewHandle>& outputScratch,
	                                  int stream,
	                                  int num_splits = 0){
        if(num_splits==0){
                num_splits = clp2((output->dim(2)*output->dim(3)-1)/65535+1);
        }
        int out_split_width = output->dim(3)/num_splits;
        if(out_split_width*num_splits != output->dim(3)){
                out_split_width += 1;
        }
        int in_split_width = filters->dim(5)+(out_split_width-1)*stepy;
        Ptr<ArrayViewHandle> input_split;
        for(int i=0;i<num_splits;i++){
                int in_split_offset = i*out_split_width*stepy;
            input_split = input->view(DDim(0,0,0,in_split_offset),DDim(input->dim(0),input->dim(1),input->dim(2),in_split_width));
                gpuCopy(input_split,inputScratch->view(DDim(i,0,0,0),input_split->dim()),stream);
                if((in_split_width+in_split_offset) > input->dim(3)){
                        break;
                }
        }
        gpuFilterTimes(filters,transpose,stepz,stepx,stepy,inputScratch,outputScratch,stream);
        Ptr<ArrayViewHandle> output_split;
        for(int i=0;i<num_splits;i++){
                if(out_split_width*(i+1) > output->dim(3)){
                        output_split = output->view(DDim(0,0,0,i*out_split_width),DDim(output->dim(0),output->dim(1),
                                output->dim(2),output->dim(3)-i*out_split_width));
                        gpuCopy(outputScratch->view(DDim(i,0,0,0),output_split->dim()),output_split,stream);
                        break;
                }else{
                        output_split = output->view(DDim(0,0,0,i*out_split_width),DDim(output->dim(0),output->dim(1),
                                output->dim(2),out_split_width));
                        gpuCopy(outputScratch->view(DDim(i,0,0,0),output_split->dim()),output_split,stream);
                }
        }
};

	static void init_filt_LCN(const Ptr<ArrayViewHandle>& filt_LCN,double mean_sigma,int stream){
        int mean_size1 = filt_LCN->dim(4);
        int mean_size2 = filt_LCN->dim(5);
        int img_dimz = filt_LCN->dim(3);
        Ptr<ArrayViewHandle> filt_vals = hostArrayAllocRM(DataType::FLOAT,DDim(mean_size1,mean_size2),stream);
        double fsum = 0.0;
        int hp = (mean_size1-1)/2;
        for(int i=0;i<mean_size1;i++){
                for(int j=0;j<mean_size2;j++){
                        int dx = i-hp;
                        int dy = j-hp;
                        double c = expf(-0.5*(dx*dx+dy*dy)/(mean_sigma*mean_sigma));
                        cpuSet(c,filt_vals->view(DDim(i,j),DDim(1,1)),stream);
                        fsum += c;
                }
        }
        cpuTimesScalar(filt_vals,1.0/fsum/img_dimz,filt_vals,stream);
        for(int i=0;i<img_dimz;i++){
                for(int j=0;j<img_dimz;j++){
                        copy(filt_vals,filt_LCN->view(DDim(i,0,0,j,0,0),DDim(1,1,1,1,mean_size1,mean_size2)),stream);
                }
        }
};

    static void init_filt_LCN_sep(const Ptr<ArrayViewHandle>& filt_LCN,double mean_sigma,int stream){
        int mean_size = filt_LCN->dim(0);
        double fsum = 0.0;
        int hp = (mean_size-1)/2;
        for(int i=0;i<mean_size;i++){
            int dx = i-hp;
            double c = expf(-0.5*(dx*dx)/(mean_sigma*mean_sigma));
            gpuSet(c,filt_LCN->view(DDim(i),DDim(1)),stream);
            fsum+=c;
        }
        gpuTimesScalar(filt_LCN,1.0/fsum,filt_LCN,stream);
    };
};
