#include "LanePredictor_honda.h"

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



LanePredictor_honda::LanePredictor_honda(int* argc, char* argv[],int stream_num){
	stream = stream_num;
	model = argv[1];
	fastCppInit(argc,&argv,stream);

	data_buf = gpuArrayAllocRM(DataType::FLOAT, DDim(BATCH_SIZE,IMG_DIMZ,IMG_DIMX,IMG_DIMY), stream);
	mean_lcn = gpuArrayAllocRM(DataType::FLOAT, DDim(BATCH_SIZE,MEAN_OUTPUTZ,MEAN_OUTPUTX,MEAN_OUTPUTY),stream);
	mean_lcn_sqr = gpuArrayAllocRM(DataType::FLOAT, mean_lcn->dim(),stream);
	divide_lcn = gpuArrayAllocRM(DataType::FLOAT, DDim(BATCH_SIZE,DIVIDE_OUTPUTZ,DIVIDE_OUTPUTX,DIVIDE_OUTPUTY),stream);
	filt_LCN = gpuArrayAllocRM(DataType::FLOAT,DDim(IMG_DIMZ,1,1,IMG_DIMZ,MEAN_SIZE1,MEAN_SIZE2),stream);
	init_filt_LCN(filt_LCN,MEAN_SIGMA,stream);

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

	allocScratchArray(data_buf,mean_lcn,filt_LCN,MEAN_STEP2,data_buf_scratch,mean_lcn_scratch,stream,MEAN_LCN_SPLITS);
	allocScratchArray(mean_lcn_sqr,divide_lcn,filt_LCN,MEAN_STEP2,mean_lcn_sqr_scratch,divide_lcn_scratch,stream,DIVIDE_LCN_SPLITS);
	allocScratchArray(divide_lcn,filt_1,W_1,FILT1_STEPY,divide_lcn_in_scratch,filt_1_scratch,stream,W1_SPLITS);
	allocScratchArray(pool_1,filt_2,W_2,FILT2_STEPY,pool_1_scratch,filt_2_scratch,stream,W2_SPLITS);
	allocScratchArray(pool_2,feat,W_3,FILT3_STEPY,pool_2_scratch,feat_scratch,stream,W3_SPLITS);
    
    host_output = hostArrayAllocRM(DataType::FLOAT,DDim(1,NUM_CLASSIFIERS),stream);

	synchronizeStream(stream);
}
LanePredictor_honda::~LanePredictor_honda(){
	fastCppShutdown();
}

Ptr<ArrayViewHandle> LanePredictor_honda::processImage(const Ptr<ArrayViewHandle>& img){
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
