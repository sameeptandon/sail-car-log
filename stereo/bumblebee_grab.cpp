#include <vector>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "bumblebee_xb3.h"

using namespace FlyCapture2;

void checkError(Error error) {
  if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
      exit(-1);
    }        
}

int main( int argc, char* argv[] ) {
  std::string outdir="./bumblebee_images/";
  mkdir(outdir.c_str(),755);

  BumblebeeXb3 cam;
  cam.format7AllCameras();
  cam.StartCapture();
  
  Image rawRight, rawCenter, rawLeft, right, center, left;
  unsigned int k_numImages = 5;
  Error error;
  for ( int imageCnt=0; imageCnt < k_numImages; imageCnt++ ) {
    
    // Retrieve an image
    cam.RetrieveBuffer( &rawRight, &rawCenter, &rawLeft );

    rawRight.Convert( PIXEL_FORMAT_RGB8, &right );
    rawCenter.Convert( PIXEL_FORMAT_RGB8, &center );
    rawLeft.Convert( PIXEL_FORMAT_RGB8, &left );
    
    printf( "Grabbed image %d\n", imageCnt );

    // Save files
    std::stringstream ss;
    ss << imageCnt;
    std::string filename = ss.str();
    checkError( left.Save( (outdir+"/"+filename+"_left.png").c_str() ) );
    checkError( center.Save( (outdir+"/"+filename+"_center.png").c_str() ) );
    checkError( right.Save( (outdir+"/"+filename+"_right.png").c_str() ) );
  }
  
  printf( "Done! \n" );
  return 0;
}
