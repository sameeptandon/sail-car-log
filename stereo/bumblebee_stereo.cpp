#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <glob.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "bumblebee_xb3.h"
#include "triclops_xb3.h"
#include "lodepng.h"
#include "pnmutils.h"

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

void rectifyImageFileName(std::string prefix) {
  
}



int main(int argc, char* argv[]) {
  std::string indir = "./bumblebee_images/";
  std::string outdir = "./stereo_short_images/";
  mkdir(outdir.c_str(),755);
  std::vector<std::string> prefixes = prefixList(glob(indir+"/*.png"), "left.png");
  
  TriclopsContext     context;
  TriclopsError       error;
  TriclopsImage16     depthImage16;
  TriclopsImage       depthImage;
  TriclopsColorImage  leftImage = {0}, centerImage = {0}, rightImage = {0};
  
  
  error=triclopsGetDefaultContextFromFile( &context, "./short.cal");
  _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", error );

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
  TriclopsInput leftData, centerData, rightData, stereoData;
  nullTriclopsInput(&leftData);
  nullTriclopsInput(&centerData);
  nullTriclopsInput(&rightData);
  nullTriclopsInput(&stereoData);  
  for (int i = 0; i < prefixes.size(); i++) {
    std::string basename = prefixes[i].substr(prefixes[i].rfind('/'), std::string::npos);        
    std::ifstream my_file((outdir+basename+"rectified_right.png").c_str());
    if (my_file.good())
      continue;

    std::cout << "Now on prefix " << prefixes[i] << std::endl;
    pngReadToTriclopsInput(prefixes[i]+"left.png", &leftData);
    pngReadToTriclopsInput(prefixes[i]+"center.png", &centerData);
    pngReadToTriclopsInput(prefixes[i]+"right.png", &rightData);    
    
    //pngReadToStereoTriclopsInputRGB(prefixes[i]+"left.png", prefixes[i]+"center.png", prefixes[i]+"right.png", &stereoData);
    //std::cout << "loading stereo..." << std::endl;

    pngReadToStereoTriclopsInputRGB(prefixes[i]+"left.png", prefixes[i]+"center.png", &stereoData);    

    //std::cout << stereoData.nrows << " " << stereoData.ncols << " " << stereoData.rowinc << " " << stereoData.inputType << std::endl;
    
    //std::cout << "Rectifying" << std::endl;
    single = clock();
    //Preprocessing the images
    error = triclopsRectify( context, &stereoData );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", error );

    //std::cout << "stereo" << std::endl;
    // Stereo processing
    error = triclopsStereo( context ) ;
    _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", error );

    //std::cout << "get disparity" << std::endl;

    // Retrieve the interpolated depth image from the context
    error = triclopsGetImage16( context, TriImg16_DISPARITY, TriCam_REFERENCE, &depthImage16 );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage16()", error );
    
    total += clock() - single;
    
    error = triclopsSaveImage16( &depthImage16, const_cast<char*>((outdir+basename+"disparity.pgm").c_str()) );    
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage()", error );

    //////////// Color rectificaiton ////////////////
    //std::cout << "get rectify color" << std::endl;

    error = triclopsRectifyColorImage( context, 
    				       TriCam_REFERENCE, 
    				       &leftData, 
    				       &leftImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", error );
    //std::cout << "saveing left.. " << std::endl;
    pngWriteFromTriclopsColorImage( outdir+basename+"rectified_left.png",
				    leftImage );


    // disparity2ptsfile( outdir+basename+"cloud.pts", context, leftImage, depthImage16 );
    
    error = triclopsRectifyColorImage( context,
    				       TriCam_LEFT,
    				       &centerData,
    				       &centerImage );

    _HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", error );
    //std::cout << "saveing center.. " << std::endl;    
    pngWriteFromTriclopsColorImage( outdir+basename+"rectified_center.png",
    				    centerImage );

    error = triclopsRectifyColorImage( context,
    				       TriCam_RIGHT,
    				       &rightData,
    				       &rightImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", error );    
    //std::cout << "saveing right.. " << std::endl;    
    pngWriteFromTriclopsColorImage( outdir+basename+"rectified_right.png",
    				    rightImage );
    
  }
  std::cout << "Avg time: " << ((double) total)/(CLOCKS_PER_SEC*prefixes.size()) << "s" << std::endl;

  // Destroy the Triclops context
  error = triclopsDestroyContext( context ) ;
  _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", error );
  
  return 0;
}
