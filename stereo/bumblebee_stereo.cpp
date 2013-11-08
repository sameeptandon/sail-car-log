#include <vector>
#include <string>
#include <queue>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <glob.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"


#include "bumblebee_xb3.h"
#include "triclops_xb3.h"
#include "lodepng.h"
#include "pnmutils.h"
#include "elas.h"
#include "BinaryIO.h"
#include "SString.h"

enum DisparityMethod {
  TRICLOPS,
  LIBELAS,
  OPENCV,
  MATLAB
};

enum OpenCvStereo {
  SBM,
  SGBM,
  GC
};

//#define PRINT( msg ) std::cout << msg << std::endl;
#define PRINT( msg ) 

#define _HANDLE_TRICLOPS_ERROR( function, error )	\
  {							\
    if( error != TriclopsErrorOk )			\
      {							\
	printf(						\
	       "ERROR: %s reported %s.\n",		\
	       function,				\
	       triclopsErrorToString( error ) );	\
	exit( 1 );					\
      }							\
  }							\

std::vector<String> glob(const String& pat){
  glob_t glob_result;
  glob(pat.c_str(),GLOB_TILDE,NULL,&glob_result);
  std::vector<String> ret;
  for(unsigned int i=0;i<glob_result.gl_pathc;++i){
    ret.push_back(String(glob_result.gl_pathv[i]));
  }
  globfree(&glob_result);
  return ret;
}

void printFiles(std::vector<String> files) {
  std::cout << files.size() << std::endl;
  for (int i = 0; i < files.size(); i++)
    std::cout << files[i] << std::endl;
}

std::vector<String> prefixList(std::vector<String> files, String postfix) {
  using namespace std;
  vector<String> prefixes;
  size_t found;
  for (int i = 0; i < files.size(); i++) {
    found = files[i].find(postfix);
    if (found == string::npos)
      continue;
    prefixes.push_back(files[i].substr(0, found));
  }
  return prefixes;
}

template <typename T>
std::queue<T> vector2queue(std::vector<T> v) {
  std::queue<T> q;
  for (int i = 0; i < v.size(); i++)
    q.push(v[i]);
  return q;
}

void rectifyColor(const TriclopsContext& context,
		  TriclopsInput* colorImage, TriclopsColorImage* leftImage,
		  TriclopsCamera cam=TriCam_REFERENCE) {
  // possible memeory leak, not sure if image data is cleared here or now
  TriclopsError error = triclopsRectifyColorImage( context, cam, colorImage, leftImage);
  _HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", error );  
}

void float2int16(const float* fBuf, const int32_t size, unsigned short* sBuf) {
  // copy float to ushort
  for (int32_t i=0; i < size; i++)
    sBuf[i] = (unsigned short) std::max(65535.0f*fBuf[i]/255.0f, 0.0f);
}

void float2int16(const float* fBuf, const int32_t* dims, unsigned short* sBuf) {
  float2int16(fBuf, dims[0]*dims[1], sBuf);
}

void opencvStereo(const TriclopsImage& left, const TriclopsImage& right,
		  TriclopsImage16* dispLeft, TriclopsImage16* dispRight,
		  OpenCvStereo ocvs, int maxDisparity = 32) {

  int width = left.ncols;
  int height = left.nrows;

  cv::Mat matLeft(height, width, CV_8UC1, left.data);
  cv::Mat matRight(height, width, CV_8UC1, right.data);
  cv::Mat matDispLeft(height, width, CV_16S);
  cv::Mat matDispRight(height, width, CV_16S);
  const int32_t dims[2] = {width,height};

  switch (ocvs) {
  case SBM: {
    // run opencv stereo
    cv::StereoBM sbm;
    sbm.state->SADWindowSize = 9;
    sbm.state->minDisparity = 0; //-39;  
    sbm.state->numberOfDisparities = maxDisparity; //112;
    sbm.state->preFilterSize = 5;
    sbm.state->preFilterCap = 61;
    sbm.state->textureThreshold = 507;
    sbm.state->uniquenessRatio = 0;
    sbm.state->speckleWindowSize = 0;
    sbm.state->speckleRange = 8;
    sbm.state->disp12MaxDiff = 1;
    sbm(matLeft, matRight, matDispLeft, CV_32F);

    // allocate data
    if (dispLeft->data) delete dispLeft->data;
    dispLeft->data = new unsigned short[left.nrows*left.ncols];

    // fill data
    float2int16((float*)matDispLeft.data, dims, dispLeft->data);
    dispLeft->nrows = right.nrows;
    dispLeft->ncols  = right.ncols;
    dispLeft->rowinc = dispLeft->ncols*2;    
  } break;
  case SGBM: {
    // run opencv stereo
    cv::StereoSGBM sgbm;
    sgbm.SADWindowSize = 9;
    sgbm.numberOfDisparities = maxDisparity;//192;
    sgbm.preFilterCap = 4;
    sgbm.minDisparity = 0; //-64; 
    sgbm.uniquenessRatio = 1;
    // sgbm.speckleWindowSize = 150;
    // sgbm.speckleRange = 2;
    sgbm.disp12MaxDiff = 20;//10;
    sgbm.fullDP = true;
    sgbm.P1 = 600;
    sgbm.P2 = 2400;
    sgbm(matLeft, matRight, matDispLeft);

    // allocate data
    if (dispLeft->data) delete dispLeft->data;
    dispLeft->data = new unsigned short[left.nrows*left.ncols];

    // fill data
    std::vector<float> dispTmp(width*height);
    for (int i = 0; i < dispTmp.size(); i++)
      dispTmp[i] = ((short*)matDispLeft.data)[i]/16.0f;
    float2int16(dispTmp.data(), dims, dispLeft->data);
    dispLeft->nrows = right.nrows;
    dispLeft->ncols  = right.ncols;
    dispLeft->rowinc = dispLeft->ncols*2;    
  } break;
  case GC: {
    CvStereoGCState* state = cvCreateStereoGCState( maxDisparity, 4 );
    CvMat matLeft2=matLeft,matRight2=matRight,matDispLeft2=matDispLeft,matDispRight2=matDispRight;
    cvFindStereoCorrespondenceGC(&matLeft2, &matRight2, &matDispLeft2, &matDispRight2, state);
    
    // allocate data
    if (dispLeft->data) delete dispLeft->data;
    dispLeft->data = new unsigned short[left.nrows*left.ncols];    
    for (int i = 0; i < width*height; i++)
      dispLeft->data[i]=(unsigned short)std::max(65535.0f*-((short*)matDispLeft.data)[i]/255.0f, 0.0f);

    // allocate data
    if (dispRight->data) delete dispRight->data;
    dispRight->data = new unsigned short[right.nrows*right.ncols];    
    for (int i = 0; i < width*height; i++)
      dispRight->data[i]=(unsigned short)std::max(65535.0f*((short*)matDispRight.data)[i]/255.0f, 0.0f);

    dispLeft->nrows = right.nrows;
    dispLeft->ncols  = right.ncols;
    dispLeft->rowinc = dispLeft->ncols*2;

    dispRight->nrows = right.nrows;
    dispRight->ncols  = right.ncols;
    dispRight->rowinc = dispRight->ncols*2;
    cvReleaseStereoGCState( &state );    
  } break;
  }

}

void libelasStereo(const TriclopsImage& left, const TriclopsImage& right,
		   TriclopsImage16* dispLeft, TriclopsImage16* dispRight,
		   bool subsampling=false) {
  static int iter = 0;
  assert(left.nrows == right.nrows);
  assert(left.ncols == right.ncols);
  assert(left.rowinc == left.ncols);
  assert(right.rowinc == right.ncols);
  int32_t width  = left.ncols;
  int32_t height = left.nrows;
  int32_t rowinc = left.rowinc;

  if (subsampling) {
    width  /= 2;
    height /= 2;
    rowinc /= 2;
  }
  
  // allocate memory for disparity images
  const int32_t dims[3] = {width,height,rowinc}; // bytes per line = width
  float* D1_data = (float*)malloc(width*height*sizeof(float));
  float* D2_data = (float*)malloc(width*height*sizeof(float));
  Elas::parameters param(Elas::MIDDLEBURY);
  param.disp_max = 50;
  param.postprocess_only_left = false;
  Elas elas(param);
  param.subsampling = subsampling;
  elas.process(left.data, right.data, D1_data, D2_data, dims);

  if (dispLeft->data) delete dispLeft->data;
  dispLeft->data = new unsigned short[width*height];
  if (dispRight->data) delete dispRight->data;
  dispRight->data = new unsigned short[width*height];
  float2int16(D1_data, dims, dispLeft->data);
  float2int16(D2_data, dims, dispRight->data);
  
  dispLeft->nrows  = height;
  dispLeft->ncols  = width;
  dispLeft->rowinc = width*2;
  
  dispRight->nrows  = height;
  dispRight->ncols  = width;
  dispRight->rowinc = width*2;  
  
  free( D1_data );
  free( D2_data );
  iter++;
}

std::vector<IplImage*> nextFrame(const std::vector<CvCapture*>& captures) {
  // next frame
  std::vector<IplImage*> frames(captures.size());
  int nFinished = 0;      
  for (int i = 0; i < frames.size(); i++) {
    frames[i] = cvQueryFrame(captures[i]);
    if (!frames[i]) nFinished++;
  }

  assert(nFinished == 0 || nFinished == frames.size());  
  if (nFinished == frames.size())
    frames.resize(0);
  return frames;
}

TriclopsInput* ipl2triclops(IplImage* iplImg) {
  assert(iplImg->depth == IPL_DEPTH_8U);
  assert(iplImg->nChannels == 3);
  assert(iplImg->dataOrder == IPL_DATA_ORDER_PIXEL);
  assert(iplImg->widthStep == 3*iplImg->width);

  TriclopsInput* triImg = new TriclopsInput;
  triImg->u.rgb32BitPacked.data = bgr2bgru(iplImg->imageData, iplImg->width, iplImg->height);
  triImg->inputType = TriInp_RGB_32BIT_PACKED;
  triImg->nrows     = iplImg->height;
  triImg->ncols     = iplImg->width;
  triImg->rowinc    = 4 * iplImg->width;
  
  return triImg;
}

void saveDepthMap(const String& floatname, const String shortname,
		  const std::vector<float>& xyz, int dims[]) {
  std::vector<float> depth(dims[1]*dims[2]);
  for (int i = 0; i < depth.size(); i++) 
    depth[i] = xyz[i*3+2];
  
  std::vector<int> floatDimsVec(dims, dims+3);
  std::vector<int> shortDimsVec(dims+1, dims+3);
  //writeBin(floatname.c_str(), xyz, floatDimsVec, FORMAT_FLOAT);
  writeBin(shortname.c_str(), depth, shortDimsVec, FORMAT_FLOAT);
}

void getRectifiedImages(TriclopsContext* context, TriclopsInput* left, TriclopsInput* right,
			TriclopsImage* rectifiedLeft, TriclopsImage* rectifiedRight) {
  
  TriclopsError error;
  TriclopsInput stereoInput = nullTriclopsInput();
  PRINT("rgb2stereo");
  rgb2stereo(*left, *right, &stereoInput);
  // Preprocessing the images
  PRINT("rectify");
  error = triclopsRectify( *context, &stereoInput );
  _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", error );

  PRINT("get image");
  triclopsGetImage( *context, TriImg_RECTIFIED, TriCam_LEFT, rectifiedLeft );
  triclopsGetImage( *context, TriImg_RECTIFIED, TriCam_RIGHT, rectifiedRight );

  PRINT("deleting");
  delete stereoInput.u.rgb.red; 
  delete stereoInput.u.rgb.green;
  PRINT("returning");
}

template <typename T>
void cropImage(const T* img, int nrows, int ncols,
	       int cmin, int cmax, int rmin, int rmax, T* cimg) {
  int pix = 0;
  for (int r = 0; r < nrows; r++) 
    if (r >= rmin && r < rmax) 
      for (int c = 0; c < ncols; c++) 
	if (c >= cmin && c < cmax) 
	  cimg[pix++] = img[c + r*ncols];
}

template <typename T>
void cropImage(const std::vector<T>& img, const std::vector<int>& dim,
	       int cmin, int cmax, int rmin, int rmax, std::vector<T>* cropped) {
  cropped->resize( (cmax-cmin) * (rmax-rmin) );
  cropImage(img.data(), dim[0], dim[1], cmin, cmax, rmin, rmax, cropped->data());
}

void cropTriclopsColorImage(const TriclopsColorImage& img,
			    int cmin, int cmax, int rmin, int rmax,
			    TriclopsColorImage* cropped) {
  
  cropped->red   = new unsigned char[(cmax-cmin) * (rmax-rmin)];
  cropped->green = new unsigned char[(cmax-cmin) * (rmax-rmin)];
  cropped->blue  = new unsigned char[(cmax-cmin) * (rmax-rmin)];

  cropped->nrows  = rmax-rmin;
  cropped->ncols  = cmax-cmin;
  cropped->rowinc = cropped->ncols;

  cropImage(img.red, img.nrows, img.ncols, cmin, cmax, rmin, rmax, cropped->red);
  cropImage(img.green, img.nrows, img.ncols, cmin, cmax, rmin, rmax, cropped->green);
  cropImage(img.blue, img.nrows, img.ncols, cmin, cmax, rmin, rmax, cropped->blue);
}

void cropTriclopsImage16(const TriclopsImage16& img,
			 int cmin, int cmax, int rmin, int rmax,
			 TriclopsImage16* cropped) {
  assert(img.rowinc == img.ncols*2);
  cropped->data = new unsigned short[(cmax-cmin)*(rmax-rmin)];
  
  cropped->nrows  = rmax-rmin;
  cropped->ncols  = cmax-cmin;
  cropped->rowinc = cropped->ncols * 2;
  
  cropImage(img.data, img.nrows, img.ncols, cmin, cmax, rmin, rmax, cropped->data);
}


int main(int argc, char* argv[]) {
  String indir = "./recordings/";
  String outdir = "./recordings_results2/";
  
  DisparityMethod method = OPENCV;
  bool upsidedown = false;
  int numCameras=3;
  int maxDisparity = 48;
  int skipCount = 250;
  bool saveDisparity = true;
  bool cropMaxDisparity = true;  
  bool shortBaseline = true;
  bool wideBaseline = true;
  std::vector<int> depthDim(2);
  //depthDim[0] = 320; depthDim[1] = 240;
  //depthDim[0] = 192; depthDim[1] = 144;
  depthDim[0] = 320; depthDim[1] = 240;
  
  mkdir(outdir.c_str(),0775);
  std::vector<String> inNames(numCameras), outNames(numCameras);
  outNames[0] = "left"; outNames[1] = "center"; outNames[2] = "right";
  inNames[1] = outNames[1];
  if (upsidedown) {
    inNames[0] = "right"; inNames[2] = "left";
  } else {
    inNames[0] = "left";  inNames[2] = "right";    
  }
  
  // get the prefixes to all the files by only looking at the left files
  std::queue<String> prefixes = vector2queue(prefixList(glob(indir+"/*.avi"), "_left.avi"));
  
  // set up triclops
  const char* shortCalFile = "./short.cal";  
  const char* wideCalFile = "./wide.cal";
  TriclopsContext     shortContext, wideContext;
  TriclopsError       error;
  
  PRINT("About to get context");
  error=triclopsGetDefaultContextFromFile( &shortContext, const_cast<char*>(shortCalFile) );
  _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", error );  
  error=triclopsGetDefaultContextFromFile( &wideContext, const_cast<char*>(wideCalFile) );  
  _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", error );
  PRINT("done gettting context");
  // Set up some stereo parameters:
  // Set to 320x240 output images
  error = triclopsSetResolution( shortContext, depthDim[1], depthDim[0] );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", error );
  error = triclopsSetResolution( wideContext, depthDim[1], depthDim[0] );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", error );  

  // Set disparity range
  error = triclopsSetDisparity( shortContext, 0, maxDisparity );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", error );  
  error = triclopsSetDisparity( wideContext, 0, maxDisparity );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", error );
  
  // error = triclopsSetDisparityMapping( context, 0, 100 );
  // _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparityMapping()", error );
  // error = triclopsSetDisparityMappingOn( context, 1 );
  // _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparityMappingOn()", error );
  
  // Lets turn off all validation except subpixel and surface
  // This works quite well
  error = triclopsSetTextureValidation( shortContext, 0 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetTextureValidation()", error );
  error = triclopsSetTextureValidation( wideContext, 0 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetTextureValidation()", error );  
  error = triclopsSetUniquenessValidation( shortContext, 0 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetUniquenessValidation()", error );
  error = triclopsSetUniquenessValidation( wideContext, 0 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetUniquenessValidation()", error );  
   
  // Turn on sub-pixel interpolation
  error = triclopsSetSubpixelInterpolation( shortContext, 1 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", error );
  error = triclopsSetSubpixelInterpolation( wideContext, 1 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", error );  
  
  clock_t timer;

  int fileInd = 0;
  while (!prefixes.empty()) {
    
    // if (strcmp(prefixes.front().c_str(), "./recordings//2013-09-13_16:44") != 0) {
    //   prefixes.pop();
    //   continue;
    // }
    
    // create output sub dir
    mkdir((outdir+"/"+prefixes.front().basename()).c_str(), 0775);
    
    // Open next video file
    std::vector<CvCapture*> captures(inNames.size());
    for (int i = 0; i < inNames.size(); i++) {
      String fileName = prefixes.front()+"_"+inNames[i]+".avi";
      captures[i] = cvCreateFileCapture(fileName.c_str());
      if (!captures[i]) {
	std::cout << "Could not find file " << fileName << std::endl;
	exit(-1);
      }
    }

    int imgInd = 0;    
    while (true) {
      // get next frame
      std::vector<IplImage*> frames = nextFrame(captures);
      if (frames.size() == 0) break;
      
      if (imgInd < skipCount) {
	imgInd++;
	continue;
      }
      
      // set output prefix
      std::stringstream ss;
      ss << outdir+"/"+prefixes.front().basename()+"/"+prefixes.front().basename()+"_" << imgInd << "_";
      String outPrefix = ss.str();
      
      // allocate triclops data
      std::vector<TriclopsInput*> inputs(frames.size()); // left, center, right
      TriclopsColorImage  shortColorImage = {0}, wideColorImage = {0};
      TriclopsImage16     shortDepthImage16Left = {0}, shortDepthImage16Right = {0};
      TriclopsImage16     wideDepthImage16Left = {0}, wideDepthImage16Right = {0};      
      TriclopsImage       shortRectifiedLeft = {0}, shortRectifiedRight = {0};
      TriclopsImage       wideRectifiedLeft = {0}, wideRectifiedRight = {0};

      // convert frames to triclops
      for (int i = 0; i < frames.size(); i++) inputs[i] = ipl2triclops(frames[i]);      
      
      PRINT("short rectified")
      if (shortBaseline)
	getRectifiedImages(&shortContext,inputs[1],inputs[2],&shortRectifiedLeft,&shortRectifiedRight);
      PRINT("wide rectified")      
      if (wideBaseline)
	getRectifiedImages(&wideContext,inputs[0],inputs[2],&wideRectifiedLeft,&wideRectifiedRight);
      
      timer = clock();
      switch (method) {
      case TRICLOPS: {
	if (shortBaseline) {
	  // Stereo processing
	  error = triclopsStereo( shortContext ) ;
	  _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", error );
	
	  // Retrieve the interpolated depth image from the context
	  error = triclopsGetImage16( shortContext, TriImg16_DISPARITY,
				      TriCam_REFERENCE, &shortDepthImage16Left );
	  _HANDLE_TRICLOPS_ERROR( "triclopsGetImage16() Left", error );
	}	
	if (wideBaseline) {
	  // Stereo processing
	  error = triclopsStereo( wideContext ) ;
	  _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", error );
	
	  // Retrieve the interpolated depth image from the context
	  error = triclopsGetImage16( wideContext, TriImg16_DISPARITY,
				      TriCam_REFERENCE, &wideDepthImage16Left );
	  _HANDLE_TRICLOPS_ERROR( "triclopsGetImage16() Left", error );
	}

      }	break;
      case LIBELAS: {
	if (shortBaseline) {
	  libelasStereo( shortRectifiedLeft, shortRectifiedRight,
			 &shortDepthImage16Left, &shortDepthImage16Right );
	}
	if (wideBaseline) {
	  libelasStereo( wideRectifiedLeft, wideRectifiedRight,
			 &wideDepthImage16Left, &wideDepthImage16Right );
	}
      }	break;
      case OPENCV: {
	PRINT("stereo short");
	if (shortBaseline) {
	  opencvStereo( shortRectifiedLeft, shortRectifiedRight,
			&shortDepthImage16Left, &shortDepthImage16Right, SGBM );
	}
	PRINT("stereo wide");	
	if (wideBaseline)  {
	  opencvStereo( wideRectifiedLeft, wideRectifiedRight,
			&wideDepthImage16Left, &wideDepthImage16Right, SGBM );

	}
      } break;
      case MATLAB: {
	// needs to rectify color images first
	// for (int i = 0; i < colorImages.size(); i++)
	//   pngWriteFromTriclopsColorImage( outPrefix+"rectified"+outNames[i]+".png", colorImages[i] );
      }break;
      }
      
      std::cout << "prefix: " << prefixes.front() << " fileInd: " << fileInd
		<< " imgInd: " << imgInd
		<< " time: " << (clock()-timer)/(double)CLOCKS_PER_SEC  << "s" << std::endl;

      
      if (shortBaseline) {
	// get depth
	std::vector<float> depth;
	disparity2depth(shortContext, shortDepthImage16Left, maxDisparity, &depth, true);
	rectifyColor(shortContext, inputs[0], &shortColorImage);
	
	// save results
	if (cropMaxDisparity) {
	  // save color image
	  TriclopsColorImage croppedImage;
	  cropTriclopsColorImage(shortColorImage, maxDisparity, depthDim[0],
				 0, depthDim[1], &croppedImage);
	  pngWriteFromTriclopsColorImage(outPrefix+"short_image.png", croppedImage);
	  freeColorImage( &croppedImage );
	  
	  // save depth map
	  std::vector<float> croppedDepth;
	  std::vector<int> croppedDepthDim(2);
	  croppedDepthDim[0] = (depthDim[0]-maxDisparity); croppedDepthDim[1] = depthDim[1];
	  cropImage(depth, depthDim, maxDisparity, depthDim[0], 0, depthDim[1], &croppedDepth);
	  writeBin((outPrefix+"short_depth.bin").c_str(), depth, croppedDepthDim, FORMAT_FLOAT);

	  // save disparity
	  if (saveDisparity) {
	    TriclopsImage16 croppedDepthImage;
	    cropTriclopsImage16(shortDepthImage16Left, maxDisparity, depthDim[0], 0, depthDim[1],
				&croppedDepthImage);
	    error=triclopsSaveImage16(&croppedDepthImage,
				      const_cast<char*>((outPrefix+"short_disparity.pgm").c_str()));
	    _HANDLE_TRICLOPS_ERROR( "tricolopsSaveImage()", error );
	    freeImage16(&croppedDepthImage);
	  }	  
	}
	else  {
	  pngWriteFromTriclopsColorImage(outPrefix+"short_image.png", shortColorImage);
	  writeBin((outPrefix+"short_depth.bin").c_str(), depth, depthDim, FORMAT_FLOAT);
	  if (saveDisparity) {
	    error=triclopsSaveImage16(&shortDepthImage16Left,
				      const_cast<char*>((outPrefix+"short_disparity.pgm").c_str()));
	    _HANDLE_TRICLOPS_ERROR( "tricolopsSaveImage()", error );
	  }	  
	}
	
	freeImage16(&shortDepthImage16Left);
	freeImage16(&shortDepthImage16Right);      
      }      

      if (wideBaseline) {
	// get depth
	std::vector<float> depth;
	disparity2depth(wideContext, wideDepthImage16Left, maxDisparity, &depth, true);
	rectifyColor(wideContext, inputs[0], &wideColorImage);
	
	// save results
	if (cropMaxDisparity) {
	  // save color image
	  TriclopsColorImage croppedImage;
	  cropTriclopsColorImage(wideColorImage, maxDisparity, depthDim[0],
				 0, depthDim[1], &croppedImage);
	  pngWriteFromTriclopsColorImage(outPrefix+"wide_image.png", croppedImage);
	  freeColorImage( &croppedImage );
	  
	  // save depth map
	  std::vector<float> croppedDepth;
	  std::vector<int> croppedDepthDim(2);
	  croppedDepthDim[0] = (depthDim[0]-maxDisparity); croppedDepthDim[1] = depthDim[1];
	  cropImage(depth, depthDim, maxDisparity, depthDim[0], 0, depthDim[1], &croppedDepth);
	  writeBin((outPrefix+"wide_depth.bin").c_str(), depth, croppedDepthDim, FORMAT_FLOAT);

	  // save disparity
	  if (saveDisparity) {
	    TriclopsImage16 croppedDepthImage;
	    cropTriclopsImage16(wideDepthImage16Left, maxDisparity, depthDim[0], 0, depthDim[1],
				&croppedDepthImage);
	    error=triclopsSaveImage16(&croppedDepthImage,
				      const_cast<char*>((outPrefix+"wide_disparity.pgm").c_str()));
	    _HANDLE_TRICLOPS_ERROR( "tricolopsSaveImage()", error );
	    freeImage16(&croppedDepthImage);
	  }	  
	}
	else  {
	  pngWriteFromTriclopsColorImage(outPrefix+"wide_image.png", wideColorImage);
	  writeBin((outPrefix+"wide_depth.bin").c_str(), depth, depthDim, FORMAT_FLOAT);
	  if (saveDisparity) {
	    error=triclopsSaveImage16(&wideDepthImage16Left,
				      const_cast<char*>((outPrefix+"wide_disparity.pgm").c_str()));
	    _HANDLE_TRICLOPS_ERROR( "tricolopsSaveImage()", error );
	  }	  
	}
	
	freeImage16(&wideDepthImage16Left);
	freeImage16(&wideDepthImage16Right);      
      }
      
      // free memory
      for (int i = 0; i < inputs.size(); i++) freeInput(inputs[i]);
      imgInd++;
    }
    fileInd++;
    prefixes.pop();    
    for (int i = 0; i < captures.size(); i++) cvReleaseCapture( &captures[i] );
  }
  
  // Destroy the Triclops context
  error = triclopsDestroyContext( shortContext ) ;
  _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", error );
  error = triclopsDestroyContext( wideContext ) ;
  _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", error );  
  
  return 0;
}
