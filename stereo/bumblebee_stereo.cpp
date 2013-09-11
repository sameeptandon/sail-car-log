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

#include "bumblebee_xb3.h"
#include "triclops_xb3.h"
#include "lodepng.h"
#include "pnmutils.h"
#include "elas.h"


using namespace cv;

enum DisparityMethod {
  TRICLOPS,
  LIBELAS,
  MATLAB
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

std::vector<std::string> glob(const std::string& pat){
  using namespace std;
  glob_t glob_result;
  glob(pat.c_str(),GLOB_TILDE,NULL,&glob_result);
  vector<string> ret;
  for(unsigned int i=0;i<glob_result.gl_pathc;++i){
    ret.push_back(string(glob_result.gl_pathv[i]));
  }
  globfree(&glob_result);
  return ret;
}

void printFiles(std::vector<std::string> files) {
  std::cout << files.size() << std::endl;
  for (int i = 0; i < files.size(); i++)
    std::cout << files[i] << std::endl;
}

std::vector<std::string> prefixList(std::vector<std::string> files, std::string postfix) {
  using namespace std;
  vector<string> prefixes;
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

void float2int16(const float* fBuf, const int32_t* dims, unsigned short* sBuf) {
  // copy float to ushort
  for (int32_t i=0; i < dims[0]*dims[1]; i++)
    sBuf[i] = (unsigned short) std::max(65535.0f*fBuf[i]/255.0f, 0.0f);
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

int main(int argc, char* argv[]) {
  std::string indir = "./";
  std::string outdir = "./stereo_images/";
  DisparityMethod method = LIBELAS;
  bool upsidedown = true;
  mkdir(outdir.c_str(),0775);
  const int numCameras=3;
  std::vector<std::string> inNames(numCameras), outNames(numCameras);
  outNames[0] = "left"; outNames[1] = "center"; outNames[2] = "right";
  inNames[1] = outNames[1];
  if (upsidedown) {
    inNames[0] = "right"; inNames[2] = "left";
  } else {
    inNames[0] = "left";  inNames[2] = "right";    
  }
  
  // get the prefixes to all the files by only looking at the left files
  std::queue<std::string> prefixes = vector2queue(prefixList(glob(indir+"/*.avi"), "left.avi"));
  
  // set up triclops
  bool wideBaseline = false;
  const char* calFile = (wideBaseline) ? "./wide.cal" : "./short.cal";
  TriclopsContext     context;
  TriclopsError       error;

  PRINT("About to get context");
  error=triclopsGetDefaultContextFromFile( &context, const_cast<char*>(calFile) );
  _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", error );
  PRINT("done gettting context");
  // Set up some stereo parameters:
  // Set to 320x240 output images
  error = triclopsSetResolution( context, 240, 320 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", error );

  // Set disparity range
  error = triclopsSetDisparity( context, 0, 100 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", error );
  
  // error = triclopsSetDisparityMapping( context, 0, 100 );
  // _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparityMapping()", error );
  // error = triclopsSetDisparityMappingOn( context, 1 );
  // _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparityMappingOn()", error );
  
  // Lets turn off all validation except subpixel and surface
  // This works quite well
  error = triclopsSetTextureValidation( context, 0 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetTextureValidation()", error );
  error = triclopsSetUniquenessValidation( context, 0 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetUniquenessValidation()", error );
   
  // Turn on sub-pixel interpolation
  error = triclopsSetSubpixelInterpolation( context, 1 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", error );
  
  clock_t single, total=0;
  int iter = 0;
  while (!prefixes.empty()) {
    // Open next video file
    std::vector<CvCapture*> captures(inNames.size());
    for (int i = 0; i < inNames.size(); i++) {
      std::string fileName = prefixes.front()+inNames[i]+".avi";
      captures[i] = cvCreateFileCapture(fileName.c_str());
      if (!captures[i]) {
	std::cout << "Could not find file " << fileName << std::endl;
	exit(-1);
      }
    }
    prefixes.pop();    
    PRINT("done getting captures");        
    while (true) {
      // get next frame
      std::vector<IplImage*> frames = nextFrame(captures);
      if (frames.size() == 0) break;

      // set output prefix
      std::stringstream ss;
      ss << outdir << iter << "_";
      std::string outPrefix = ss.str();
      std::cout << "out prefix: " << outPrefix << std::endl;

      // allocate triclops data
      std::vector<TriclopsInput*> inputs(frames.size()); // left, center, right
      std::vector<TriclopsColorImage>  colorImages(inputs.size());
      for (int i = 0; i < colorImages.size(); i++) colorImages[i] = {0};      
      TriclopsInput       stereoInput = nullTriclopsInput(); 
      TriclopsImage16     depthImage16Left = {0}, depthImage16Right = {0};
      TriclopsImage       depthImage = {0}, rectifiedLeft = {0}, rectifiedRight = {0};
      
      // convert frames to triclops
      for (int i = 0; i < frames.size(); i++) inputs[i] = ipl2triclops(frames[i]);
      
      if (wideBaseline)
	rgb2stereo(*inputs[0], *inputs[2], &stereoInput);      
      else
	rgb2stereo(*inputs[1], *inputs[2], &stereoInput);
      
      // Preprocessing the images
      error = triclopsRectify( context, &stereoInput );
      _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", error );
      
      triclopsGetImage( context, TriImg_RECTIFIED, TriCam_LEFT, &rectifiedLeft );
      triclopsGetImage( context, TriImg_RECTIFIED, TriCam_RIGHT, &rectifiedRight );
      
      single = clock();
      switch (method) {
      case TRICLOPS: {
	// Stereo processing
	error = triclopsStereo( context ) ;
	_HANDLE_TRICLOPS_ERROR( "triclopsStereo()", error );
	
	// Retrieve the interpolated depth image from the context
	error = triclopsGetImage16( context, TriImg16_DISPARITY, TriCam_LEFT, &depthImage16Left );
	_HANDLE_TRICLOPS_ERROR( "triclopsGetImage16()", error );	
	error = triclopsGetImage16( context, TriImg16_DISPARITY, TriCam_RIGHT, &depthImage16Right );
	_HANDLE_TRICLOPS_ERROR( "triclopsGetImage16()", error );

      }	break;
      case LIBELAS: {
	PRINT("about to get left image");
	PRINT("got right image");		
	libelasStereo( rectifiedLeft, rectifiedRight, &depthImage16Left, &depthImage16Right );
	PRINT("got stereo image");
	// if (rectifiedLeft.data) free( (char*) rectifiedLeft.data );
	// PRINT("deleting right stereo image");	
	// if (rectifiedRight.data) free( (char*) rectifiedRight.data );
	// freeImage(&rectifiedLeft);
	// freeImage(&rectifiedRight);
	PRINT("Done freeing image");
      }	break;
      case MATLAB: {
	for (int i = 0; i < colorImages.size(); i++)
	  pngWriteFromTriclopsColorImage( outPrefix+"rectified"+outNames[i]+".png", colorImages[i] );
      }break;
      }
      
      total += clock() - single;
      PRINT("Saving depth map");
      error = triclopsSaveImage16( &depthImage16Left,
				   const_cast<char*>((outPrefix+"disparity_left.pgm").c_str()) );
      error = triclopsSaveImage16( &depthImage16Right,
				   const_cast<char*>((outPrefix+"disparity_right.pgm").c_str()) );      
      _HANDLE_TRICLOPS_ERROR( "tricolopsSaveImage()", error );

      // get rectified images
      for (int i = 0; i < inputs.size(); i++) {
	rectifyColor(context, inputs[i], &colorImages[i]);
	pngWriteFromTriclopsColorImage(outPrefix+"rectified_"+outNames[i]+".png", colorImages[i]);	
      }

      pngWriteFromTriclopsImage(outPrefix+"grey_rectified_"+outNames[0]+".png", rectifiedLeft);
      pngWriteFromTriclopsImage(outPrefix+"grey_rectified_"+outNames[1]+".png", rectifiedRight);
      
      PRINT("done saving depth map");      
      // disparity2ptsfile( outdir+basename+"cloud.pts", context, leftImage, depthImage16 );

      PRINT("Freeing data");            
      delete stereoInput.u.rgb.red; 
      delete stereoInput.u.rgb.green; 
      
      PRINT("Freeing data");
      for (int i = 0; i < inputs.size(); i++) freeInput(inputs[i]);
      PRINT("Freeing data");      
      //for (int i = 0; i < colorImages.size(); i++) freeColorImage(&colorImages[i]);
      PRINT("Freeing data");      
      freeImage16(&depthImage16Left);
      freeImage16(&depthImage16Right);      
      PRINT("Freeing data");      
      freeImage(&depthImage);
      // PRINT("Free left");
      // freeImage(&rectifiedLeft);
      // PRINT("Free right");      
      // freeImage(&rectifiedRight);      
      // PRINT("Done freeing data"); 
      iter++;
    }
    for (int i = 0; i < captures.size(); i++) cvReleaseCapture( &captures[i] );
  }
  
  // Destroy the Triclops context
  error = triclopsDestroyContext( context ) ;
  _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", error );
  
  return 0;
}
