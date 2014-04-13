#include "FastCpp.h"
//#include "gpu/kernels/GpuKernels.h"
#include <fstream>
#include <iostream>
#include "gpu/GpuBlockOps.h"
#include "gpu/GpuMath.h"
#include "io/ArrayIO.h"
#include "cuda_profiler_api.h"

#define BATCH_SIZE 1
#define IMG_DIMZ 3
#define IMG_DIMX 640
#define IMG_DIMY 480

#define MEAN_AXIS1 2
#define MEAN_AXIS2 3
#define MEAN_SIZE1 9
#define MEAN_SIZE2 9
#define MEAN_SIGMA ((MEAN_SIZE1-1)/4.0)
#define MEAN_STEP1 1
#define MEAN_STEP2 1
#define MEAN_OUTPUTZ IMG_DIMZ
#define MEAN_OUTPUTX ((IMG_DIMX-MEAN_SIZE1)/MEAN_STEP1+1)
#define MEAN_OUTPUTY ((IMG_DIMY-MEAN_SIZE2)/MEAN_STEP2+1)
#define MEAN_POS DDim(0,(IMG_DIMZ-MEAN_OUTPUTZ)/2,(IMG_DIMX-MEAN_OUTPUTX)/2,(IMG_DIMY-MEAN_OUTPUTY)/2)
#define DIVIDE_OUTPUTZ IMG_DIMZ
#define DIVIDE_OUTPUTX ((MEAN_OUTPUTX-MEAN_SIZE1)/MEAN_STEP1+1)
#define DIVIDE_OUTPUTY ((MEAN_OUTPUTY-MEAN_SIZE2)/MEAN_STEP2+1)
#define DIVIDE_POS DDim(0,(MEAN_OUTPUTZ-DIVIDE_OUTPUTZ)/2,(MEAN_OUTPUTX-DIVIDE_OUTPUTX)/2,(MEAN_OUTPUTY-DIVIDE_OUTPUTY)/2)
#define DIVIDE_EPS 500.0	

#define MAPS_1 16
#define FILT1_DIMZ 3
#define FILT1_DIMX 10
#define FILT1_DIMY 10
#define FILT1_STEPZ 3
#define FILT1_STEPX 2
#define FILT1_STEPY 2
#define FILT1_OUTPUTZ (((DIVIDE_OUTPUTZ-FILT1_DIMZ)/FILT1_STEPZ+1)*MAPS_1)
#define FILT1_OUTPUTX ((DIVIDE_OUTPUTX-FILT1_DIMX)/FILT1_STEPX+1)
#define FILT1_OUTPUTY ((DIVIDE_OUTPUTY-FILT1_DIMY)/FILT1_STEPY+1)
#define THRESHOLD_NL "fmaxf(.1f*a,a)"
#define SUBSAMP1_DIMZ MAPS_1
#define SUBSAMP1_DIMX 2
#define SUBSAMP1_DIMY 2
#define SUBSAMP1_STEPZ MAPS_1
#define SUBSAMP1_STEPX 2
#define SUBSAMP1_STEPY 2
#define SUBSAMP1_OUTPUTZ (((FILT1_OUTPUTZ-SUBSAMP1_DIMZ)/SUBSAMP1_STEPZ+1)*MAPS_1)
#define SUBSAMP1_OUTPUTX ((FILT1_OUTPUTX-SUBSAMP1_DIMX)/SUBSAMP1_STEPX+1)
#define SUBSAMP1_OUTPUTY ((FILT1_OUTPUTY-SUBSAMP1_DIMY)/SUBSAMP1_STEPY+1)

#define MAPS_2 16
#define FILT2_DIMZ MAPS_1
#define FILT2_DIMX 9
#define FILT2_DIMY 9
#define FILT2_STEPZ MAPS_2
#define FILT2_STEPX 1
#define FILT2_STEPY 1
#define FILT2_OUTPUTZ (((SUBSAMP1_OUTPUTZ-FILT2_DIMZ)/FILT2_STEPZ+1)*MAPS_2)
#define FILT2_OUTPUTX ((SUBSAMP1_OUTPUTX-FILT2_DIMX)/FILT2_STEPX+1)
#define FILT2_OUTPUTY ((SUBSAMP1_OUTPUTY-FILT2_DIMY)/FILT2_STEPY+1)
#define SUBSAMP2_DIMZ MAPS_2
#define SUBSAMP2_DIMX 2
#define SUBSAMP2_DIMY 2
#define SUBSAMP2_STEPZ MAPS_2
#define SUBSAMP2_STEPX 2
#define SUBSAMP2_STEPY 2
#define SUBSAMP2_OUTPUTZ (((FILT2_OUTPUTZ-SUBSAMP2_DIMZ)/SUBSAMP2_STEPZ+1)*MAPS_2)
#define SUBSAMP2_OUTPUTX ((FILT2_OUTPUTX-SUBSAMP2_DIMX)/SUBSAMP2_STEPX+1)
#define SUBSAMP2_OUTPUTY ((FILT2_OUTPUTY-SUBSAMP2_DIMY)/SUBSAMP2_STEPY+1)

#define MAPS_3 32
#define FILT3_DIMZ MAPS_2
#define FILT3_DIMX 8
#define FILT3_DIMY 8
#define FILT3_STEPZ MAPS_3
#define FILT3_STEPX 1
#define FILT3_STEPY 1
#define FILT3_OUTPUTZ (((SUBSAMP2_OUTPUTZ-FILT3_DIMZ)/FILT3_STEPZ+1)*MAPS_3)
#define FILT3_OUTPUTX ((SUBSAMP2_OUTPUTX-FILT3_DIMX)/FILT3_STEPX+1)
#define FILT3_OUTPUTY ((SUBSAMP2_OUTPUTY-FILT3_DIMY)/FILT3_STEPY+1)

#define BOTTLENECK 480

#define NUM_CLASSES 80
#define NUM_CLASSIFIERS 48
#define MIN_ACTIVATION 1.0e-8
#define MAX_ACTIVATION 1.0-1.0e-8

#define SUBPIXEL_WINDOW 7

#define MEAN_LCN_SPLITS 16
#define DIVIDE_LCN_SPLITS 16
#define W1_SPLITS 16
#define W2_SPLITS 16
#define W3_SPLITS 16

class LanePredictor{

public:
	LanePredictor();
	LanePredictor(int* argc, char** argv, int stream_num);
	~LanePredictor();
	Ptr<ArrayViewHandle> processImage(const Ptr<ArrayViewHandle>& img);

private:
	int stream;
	std::string model;
	Ptr<ArrayViewHandle> data_buf;
	Ptr<ArrayViewHandle> mean_lcn;
	Ptr<ArrayViewHandle> mean_lcn_sqr;
	Ptr<ArrayViewHandle> divide_lcn;
	Ptr<ArrayViewHandle> filt_LCN;
	Ptr<ArrayViewHandle> W_1;
	Ptr<ArrayViewHandle> filt_1;
	Ptr<ArrayViewHandle> nonlin_1;
	Ptr<ArrayViewHandle> pool_1;
	Ptr<ArrayViewHandle> W_2;
	Ptr<ArrayViewHandle> filt_2;
	Ptr<ArrayViewHandle> nonlin_2;
	Ptr<ArrayViewHandle> pool_2;
	Ptr<ArrayViewHandle> W_3;
	Ptr<ArrayViewHandle> feat;
	Ptr<ArrayViewHandle> W_bn;
	Ptr<ArrayViewHandle> B_bn;
	Ptr<ArrayViewHandle> feat_bn;
	Ptr<ArrayViewHandle> W_final;
	Ptr<ArrayViewHandle> B_final;
	Ptr<ArrayViewHandle> mult_linear;
	Ptr<ArrayViewHandle> reduce_col;
	Ptr<ArrayViewHandle> indexArray;
	Ptr<ArrayViewHandle> tmpArray;
	Ptr<ArrayViewHandle> labels;
	Ptr<ArrayViewHandle> data_buf_scratch;
	Ptr<ArrayViewHandle> mean_lcn_scratch;
	Ptr<ArrayViewHandle> mean_lcn_sqr_scratch;
	Ptr<ArrayViewHandle> divide_lcn_scratch;
	Ptr<ArrayViewHandle> filt_1_scratch;
	Ptr<ArrayViewHandle> divide_lcn_in_scratch;
	Ptr<ArrayViewHandle> pool_1_scratch;
	Ptr<ArrayViewHandle> filt_2_scratch;
	Ptr<ArrayViewHandle> pool_2_scratch;
	Ptr<ArrayViewHandle> feat_scratch;
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

	void allocScratchArray(  const Ptr<ArrayViewHandle>& input,
	                         const Ptr<ArrayViewHandle>& output, 
	                         const Ptr<ArrayViewHandle>& filters,
	                         int stepx,
	                         Ptr<ArrayViewHandle>& inputScratch,
	                         Ptr<ArrayViewHandle>& outputScratch,
	                         int stream,
	                         int num_splits = 0);

	static void gpuFilterTimesLarge(  const Ptr<ArrayViewHandle>& filters,
	                                  bool transpose,
	                                  int stepz, int stepx, int stepy,
	                                  const Ptr<ArrayViewHandle>& input,
        	                          const Ptr<ArrayViewHandle>& output,
	                                  const Ptr<ArrayViewHandle>& inputScratch,
	                                  const Ptr<ArrayViewHandle>& outputScratch,
	                                  int stream,
	                                  int num_splits = 0);

	void init_filt_LCN();
};
