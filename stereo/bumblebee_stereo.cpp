#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <glob.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "bumblebee_xb3.h"
#include "triclops_xb3.h"
#include "lodepng.h"

#define _HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 triclopsErrorToString( error ) ); \
      exit( 1 ); \
   } \
} \

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
  std::string indir = "./bumblebee_images";
  std::string rectifiedDir = "./rectified_images";
  mkdir(rectifiedDir.c_str(),755);
  std::vector<std::string> prefixes = prefixList(glob(indir+"/*.png"), "left.png");
  
  TriclopsContext     context;
  TriclopsError       error;
  TriclopsImage16     depthImage16;
  TriclopsColorImage  colorImage = {0};
  
  error=triclopsGetDefaultContextFromFile( &context, "./wide.cal");
  _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", error );

  // Set up some stereo parameters:
  // Set to 320x240 output images
  error = triclopsSetResolution( context, 240, 320 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", error );
   
  // Set disparity range
  error = triclopsSetDisparity( context, 0, 100 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", error );   
   
  // Lets turn off all validation except subpixel and surface
  // This works quite well
  error = triclopsSetTextureValidation( context, 0 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetTextureValidation()", error );
  error = triclopsSetUniquenessValidation( context, 0 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetUniquenessValidation()", error );
   
  // Turn on sub-pixel interpolation
  error = triclopsSetSubpixelInterpolation( context, 1 );
  _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", error );  

  TriclopsInput colorData, stereoData;
  nullTriclopsInput(&colorData);
  nullTriclopsInput(&stereoData);  
  for (int i = 0; i < prefixes.size(); i++) {
    std::cout << "Now on prefix " << prefixes[i] << std::endl;
    pngReadToTriclopsInput(prefixes[i]+"left.png", &colorData);
    pngReadToStereoTriclopsInputRGB(prefixes[i]+"left.png", prefixes[i]+"center.png", prefixes[i]+"right.png", &stereoData);
    
    // Preprocessing the images
    error = triclopsRectify( context, &stereoData );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", error );

    // Retrieve the interpolated depth image from the context
    error = triclopsGetImage16( context, 
				TriImg16_DISPARITY, 
				TriCam_REFERENCE, 
				&depthImage16 );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage16()", error );

    error = triclopsRectifyColorImage( context, 
				       TriCam_REFERENCE, 
				       &colorData, 
				       &colorImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", error );    
  }

  // Destroy the Triclops context
  error = triclopsDestroyContext( context ) ;
  _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", error );
  
  return 0;
}
