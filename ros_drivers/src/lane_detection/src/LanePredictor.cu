#include "FastCpp.h"
#include "gpu/kernels/GpuKernels.h"
#include <fstream>
#include <iostream>
#include "gpu/GpuBlockOps.h"
#include "gpu/GpuMath.h"
#include "io/ArrayIO.h"
#include "cuda_profiler_api.h"
#include "LanePredictor.h"

LanePredictor::LanePredictor(int* argc, char* argv[],int stream_num){
	stream = stream_num;
	model = argv[1];
	fastCppInit(argc,&argv,stream);

	data_buf = gpuArrayAllocRM(DataType::FLOAT, DDim(BATCH_SIZE,IMG_DIMZ,IMG_DIMX,IMG_DIMY), stream);
	mean_lcn = gpuArrayAllocRM(DataType::FLOAT, DDim(BATCH_SIZE,MEAN_OUTPUTZ,MEAN_OUTPUTX,MEAN_OUTPUTY),stream);
	mean_lcn_sqr = gpuArrayAllocRM(DataType::FLOAT, mean_lcn->dim(),stream);
	divide_lcn = gpuArrayAllocRM(DataType::FLOAT, DDim(BATCH_SIZE,DIVIDE_OUTPUTZ,DIVIDE_OUTPUTX,DIVIDE_OUTPUTY),stream);
	filt_LCN = gpuArrayAllocRM(DataType::FLOAT,DDim(IMG_DIMZ,1,1,IMG_DIMZ,MEAN_SIZE1,MEAN_SIZE2),stream);
	init_filt_LCN();

	W_1 = gpuArrayAllocRM(DataType::FLOAT, DDim(MAPS_1,1,1,FILT1_DIMZ,FILT1_DIMX,FILT1_DIMY), stream);
	Ptr<DistArrayHandle> W_1_load = loadDistArray(model+"_W_1",DDim(1,1,1,1,1,1),stream);
	copy(W_1_load->localHandle(),W_1,stream);
	filt_1 = gpuArrayAllocRM(DataType::FLOAT, DDim(BATCH_SIZE,FILT1_OUTPUTZ,FILT1_OUTPUTX,FILT1_OUTPUTY), stream);
	nonlin_1 = gpuArrayAllocRM(DataType::FLOAT, filt_1->dim(),stream);
	pool_1 = gpuArrayAllocRM(DataType::FLOAT, DDim(BATCH_SIZE,SUBSAMP1_OUTPUTZ,SUBSAMP1_OUTPUTX,SUBSAMP1_OUTPUTY),stream);

	W_2 = gpuArrayAllocRM(DataType::FLOAT, DDim(MAPS_2,1,1,FILT2_DIMZ,FILT2_DIMX,FILT2_DIMY),stream);
	Ptr<DistArrayHandle> W_2_load = loadDistArray(model+"_W_2",DDim(1,1,1,1,1,1),stream);
	copy(W_2_load->localHandle(),W_2,stream);
	filt_2 = gpuArrayAllocRM(DataType::FLOAT, DDim(BATCH_SIZE,FILT2_OUTPUTZ,FILT2_OUTPUTX,FILT2_OUTPUTY),stream);
	nonlin_2 = gpuArrayAllocRM(DataType::FLOAT, filt_2->dim(),stream);
	pool_2 = gpuArrayAllocRM(DataType::FLOAT, DDim(BATCH_SIZE,SUBSAMP2_OUTPUTZ,SUBSAMP2_OUTPUTX,SUBSAMP2_OUTPUTY),stream);

	W_3 = gpuArrayAllocRM(DataType::FLOAT, DDim(MAPS_3,1,1,FILT3_DIMZ,FILT3_DIMX,FILT3_DIMY),stream);
	Ptr<DistArrayHandle> W_3_load = loadDistArray(model+"_W_3",DDim(1,1,1,1,1,1),stream);
	copy(W_3_load->localHandle(),W_3,stream);
	feat = gpuArrayAllocRM(DataType::FLOAT, DDim(BATCH_SIZE,FILT3_OUTPUTZ,FILT3_OUTPUTX,FILT3_OUTPUTY),stream);

	W_bn = gpuArrayAllocRM(DataType::FLOAT,DDim(BOTTLENECK,feat->dim(1),feat->dim(2),feat->dim(3)),stream);
	Ptr<DistArrayHandle> W_bn_load = loadDistArray(model+"_W_bn",DDim(1,1,1,1),stream);
	copy(W_bn_load->localHandle(),W_bn,stream);
	B_bn = gpuArrayAllocRM(DataType::FLOAT,DDim(1,BOTTLENECK),stream);
	Ptr<DistArrayHandle> B_bn_load = loadDistArray(model+"_B_bn",DDim(1,1,1,1),stream);
	copy(B_bn_load->localHandle(),B_bn,stream);
	feat_bn = gpuArrayAllocRM(DataType::FLOAT,DDim(BATCH_SIZE,BOTTLENECK),stream);

	W_final = gpuArrayAllocRM(DataType::FLOAT,DDim(NUM_CLASSES*NUM_CLASSIFIERS,BOTTLENECK),stream);
	Ptr<DistArrayHandle> W_final_load = loadDistArray(model+"_W_final",DDim(1,1,1,1),stream);
	copy(W_final_load->localHandle(),W_final,stream);
	B_final = gpuArrayAllocRM(DataType::FLOAT,DDim(1,W_final->dim(0)),stream);
	Ptr<DistArrayHandle> B_final_load = loadDistArray(model+"_B_final",DDim(1,1,1,1),stream);
	copy(B_final_load->localHandle(),B_final,stream);
	mult_linear = gpuArrayAllocRM(DataType::FLOAT,DDim(BATCH_SIZE,W_final->dim(0)),stream);

	reduce_col = gpuArrayAllocRM(DataType::FLOAT,DDim(BATCH_SIZE,1,NUM_CLASSIFIERS),stream);
	indexArray = gpuArrayAllocRM(DataType::FLOAT,DDim(BATCH_SIZE,NUM_CLASSES,NUM_CLASSIFIERS),stream);
	for(int i=0;i<NUM_CLASSES;i++){
		gpuSet(i,indexArray->view(DDim(0,i,0),DDim(BATCH_SIZE,1,NUM_CLASSIFIERS)),stream);
	}
	tmpArray = gpuArrayAllocRM(DataType::FLOAT,indexArray->dim(),stream);
	labels = gpuArrayAllocRM(DataType::FLOAT,reduce_col->dim(),stream);

	allocScratchArray(data_buf,mean_lcn,filt_LCN,MEAN_STEP1,data_buf_scratch,mean_lcn_scratch,stream,MEAN_LCN_SPLITS);
	allocScratchArray(mean_lcn_sqr,divide_lcn,filt_LCN,MEAN_STEP1,mean_lcn_sqr_scratch,divide_lcn_scratch,stream,DIVIDE_LCN_SPLITS);
	allocScratchArray(divide_lcn,filt_1,W_1,FILT1_STEPX,divide_lcn_in_scratch,filt_1_scratch,stream,W1_SPLITS);
	allocScratchArray(pool_1,filt_2,W_2,FILT2_STEPX,pool_1_scratch,filt_2_scratch,stream,W2_SPLITS);
	allocScratchArray(pool_2,feat,W_3,FILT3_STEPX,pool_2_scratch,feat_scratch,stream,W3_SPLITS);
    
    host_output = hostArrayAllocRM(DataType::FLOAT,DDim(1,NUM_CLASSIFIERS),stream);

	synchronizeStream(stream);
}
LanePredictor::~LanePredictor(){
	fastCppShutdown();
}

Ptr<ArrayViewHandle> LanePredictor::processImage(const Ptr<ArrayViewHandle>& img){
    copy(img,data_buf,stream);
	gpuFilterTimesLarge(filt_LCN,false,IMG_DIMZ,MEAN_STEP1,MEAN_STEP2,data_buf,mean_lcn,
		data_buf_scratch,mean_lcn_scratch,stream,MEAN_LCN_SPLITS);
	gpuMinus(data_buf->view(MEAN_POS,mean_lcn->dim()),mean_lcn,mean_lcn,stream);

	gpuSquare(mean_lcn,mean_lcn_sqr,stream);
	gpuFilterTimesLarge(filt_LCN,false,IMG_DIMZ,MEAN_STEP1,MEAN_STEP2,mean_lcn_sqr,divide_lcn,
		mean_lcn_sqr_scratch,divide_lcn_scratch,stream,DIVIDE_LCN_SPLITS);
	gpuPlusScalar(divide_lcn,DIVIDE_EPS,divide_lcn,stream);
	gpuSqrt(divide_lcn,divide_lcn,stream);
	gpuDivide(mean_lcn->view(DIVIDE_POS,divide_lcn->dim()),divide_lcn,divide_lcn,stream);

	gpuFilterTimesLarge(W_1,false,FILT1_STEPZ,FILT1_STEPX,FILT1_STEPY,divide_lcn,filt_1,
		divide_lcn_in_scratch,filt_1_scratch,stream,W1_SPLITS);
	gpuBinaryOp(filt_1,filt_1,nonlin_1,THRESHOLD_NL,stream);
	gpuLocalSum2D(nonlin_1,2,3,SUBSAMP1_DIMX,SUBSAMP1_DIMY,SUBSAMP1_STEPX,SUBSAMP1_STEPY,pool_1,stream);
	gpuTimesScalar(pool_1,1.0/(SUBSAMP1_DIMX*SUBSAMP1_DIMY),pool_1,stream);

	gpuFilterTimesLarge(W_2,false,FILT2_STEPZ,FILT2_STEPX,FILT2_STEPY,pool_1,filt_2,pool_1_scratch,filt_2_scratch,stream,W2_SPLITS);
	gpuBinaryOp(filt_2,filt_2,nonlin_2,THRESHOLD_NL,stream);
	gpuLocalSum2D(nonlin_2,2,3,SUBSAMP2_DIMX,SUBSAMP2_DIMY,SUBSAMP2_STEPX,SUBSAMP2_STEPY,pool_2,stream);
	gpuTimesScalar(pool_2,1.0/(SUBSAMP2_DIMX*SUBSAMP2_DIMY),pool_2,stream);

	gpuFilterTimesLarge(W_3,false,FILT3_STEPZ,FILT3_STEPX,FILT3_STEPY,pool_2,feat,pool_2_scratch,feat_scratch,stream,W3_SPLITS);

	gpuGEMM('n','t',1.0f,reshape(feat,DDim(BATCH_SIZE,feat->dim(1)*feat->dim(2)*feat->dim(3)),stream),
		reshape(W_bn,DDim(BOTTLENECK,W_bn->dim(1)*W_bn->dim(2)*W_bn->dim(3)),stream),0.0f,feat_bn,stream);
	gpuPlus(feat_bn,B_bn,feat_bn,stream);

	gpuGEMM('n','t',1.0f,feat_bn,W_final,0.0f,mult_linear,stream);
	gpuPlus(mult_linear,B_final,mult_linear,stream);
	Ptr<ArrayViewHandle> prob = reshape(mult_linear,DDim(BATCH_SIZE,NUM_CLASSES,NUM_CLASSIFIERS),stream);

	gpuMax(prob,1,reduce_col,stream);
	gpuMinus(prob,reduce_col,prob,stream);
	gpuExp(prob,prob,stream);
	gpuUnaryOp(prob,MIN_ACTIVATION,prob,"fmaxf(a,b)",stream);
	gpuSum(prob,1,reduce_col,stream);
	gpuDivide(prob,reduce_col,prob,stream);
	gpuUnaryOp(prob,MIN_ACTIVATION,prob,"fmaxf(a,b)",stream);
	gpuUnaryOp(prob,MAX_ACTIVATION,prob,"fminf(a,b)",stream);

	//find pixel output
	gpuMax(prob,1,reduce_col,stream);
	gpuEquals(prob,reduce_col,tmpArray,stream);
	gpuTimes(indexArray,tmpArray,tmpArray,stream);
	gpuMax(tmpArray,1,labels,stream);
	gpuPlusScalar(labels,-SUBPIXEL_WINDOW/2,reduce_col,stream);
	gpuGreaterEqual(indexArray,reduce_col,tmpArray,stream);
	gpuTimes(tmpArray,prob,prob,stream);
	gpuPlusScalar(labels,SUBPIXEL_WINDOW/2,reduce_col,stream);
	gpuLessEqual(indexArray,reduce_col,tmpArray,stream);
	gpuTimes(prob,tmpArray,prob,stream);
	gpuSum(prob,1,reduce_col,stream);
	gpuTimes(prob,indexArray,prob,stream);
	gpuSum(prob,1,labels,stream);
	gpuDivide(labels,reduce_col,labels,stream); 

	Ptr<ArrayViewHandle> label_output = reshape(labels,DDim(1,NUM_CLASSIFIERS),stream);
    copy(label_output,host_output,stream);

	return host_output;
}
void LanePredictor::init_filt_LCN(){
	Ptr<ArrayViewHandle> filt_vals = hostArrayAllocRM(DataType::FLOAT,DDim(MEAN_SIZE1,MEAN_SIZE2),stream);
	double fsum = 0.0;
	int hp = (MEAN_SIZE1-1)/2;
	for(int i=0;i<MEAN_SIZE1;i++){
		for(int j=0;j<MEAN_SIZE2;j++){
			int dx = i-hp;
			int dy = j-hp;
			double c = expf(-0.5*(dx*dx+dy*dy)/(MEAN_SIGMA*MEAN_SIGMA));
			cpuSet(c,filt_vals->view(DDim(i,j),DDim(1,1)),stream);
			fsum += c;
		}
	}
	cpuTimesScalar(filt_vals,1.0/fsum/IMG_DIMZ,filt_vals,stream);
	for(int i=0;i<IMG_DIMZ;i++){
		for(int j=0;j<IMG_DIMZ;j++){
			copy(filt_vals,filt_LCN->view(DDim(i,0,0,j,0,0),DDim(1,1,1,1,MEAN_SIZE1,MEAN_SIZE2)),stream);
		}
	}
}

void LanePredictor::allocScratchArray(  const Ptr<ArrayViewHandle>& input,
                                        const Ptr<ArrayViewHandle>& output,
                                        const Ptr<ArrayViewHandle>& filters,
                                        int stepx,
                                        Ptr<ArrayViewHandle>& inputScratch,
                                        Ptr<ArrayViewHandle>& outputScratch,
                                        int stream,
                                        int num_splits){
	if(num_splits==0){
		num_splits = clp2((output->dim(2)*output->dim(3)-1)/65535+1);
	}
	int split_width = output->dim(2)/num_splits;
	if(split_width*num_splits != output->dim(2)){
		split_width += 1;
	}
	inputScratch =  gpuArrayAllocRM(input->dataType(),DDim(num_splits,input->dim(1),filters->dim(4)+(split_width-1)*stepx,
		input->dim(3)),stream);
	outputScratch = gpuArrayAllocRM(output->dataType(),DDim(num_splits,output->dim(1),split_width,output->dim(3)),stream);
}

void LanePredictor::gpuFilterTimesLarge(  const Ptr<ArrayViewHandle>& filters,
                                          bool transpose,
                                          int stepz, int stepx, int stepy,
                                          const Ptr<ArrayViewHandle>& input,
                                          const Ptr<ArrayViewHandle>& output,
                                          const Ptr<ArrayViewHandle>& inputScratch,
                                          const Ptr<ArrayViewHandle>& outputScratch,
                                          int stream,
                                          int num_splits){
	if(num_splits==0){
		num_splits = clp2((output->dim(2)*output->dim(3)-1)/65535+1);
	}
	int out_split_width = output->dim(2)/num_splits;
	if(out_split_width*num_splits != output->dim(2)){
		out_split_width += 1;
	}
	int in_split_width = filters->dim(4)+(out_split_width-1)*stepx;
	Ptr<ArrayViewHandle> input_split;
	for(int i=0;i<num_splits;i++){
		int in_split_offset = i*out_split_width*stepx;
	  input_split = input->view(DDim(0,0,in_split_offset,0),DDim(input->dim(0),input->dim(1),in_split_width,input->dim(3)));
		gpuCopy(input_split,inputScratch->view(DDim(i,0,0,0),input_split->dim()),stream);
		if((in_split_width+in_split_offset) > input->dim(2)){
			break;
		}
	}
	gpuFilterTimes(filters,transpose,stepz,stepx,stepy,inputScratch,outputScratch,stream);
	Ptr<ArrayViewHandle> output_split;
	for(int i=0;i<num_splits;i++){
		if(out_split_width*(i+1) > output->dim(2)){
			output_split = output->view(DDim(0,0,i*out_split_width,0),DDim(output->dim(0),output->dim(1),
				output->dim(2)-i*out_split_width,output->dim(3)));
			gpuCopy(outputScratch->view(DDim(i,0,0,0),output_split->dim()),output_split,stream);
			break;
		}else{
			output_split = output->view(DDim(0,0,i*out_split_width,0),DDim(output->dim(0),output->dim(1),
				out_split_width,output->dim(3)));
			gpuCopy(outputScratch->view(DDim(i,0,0,0),output_split->dim()),output_split,stream);
		}
	}
}


