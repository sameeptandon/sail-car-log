#include <vector>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include "bumblebee_xb3.h"
#include "triclops_xb3.h"

using namespace FlyCapture2;

int main(int /*argc*/, char** /*argv*/) {

  BumblebeeXb3 cam;
  cam.format7AllCameras();
  cam.StartCapture();

  Image right, center, left;
  unsigned int k_numImages = 5;
  for ( int imageCnt=0; imageCnt < k_numImages; imageCnt++ ) {
    
    // Retrieve an image
    cam.RetrieveBuffer( &right, &center, &left );
    
    printf( "Grabbed image %d\n", imageCnt );

    // Save files
    std::stringstream ss;
    ss << imageCnt;
    std::string filename = ss.str();
    left.Save( ("left-"+filename+".ppm").c_str() );
    center.Save( ("center-"+filename+".ppm").c_str() );
    right.Save( ("right-"+filename+".ppm").c_str() );
  }
  
  printf( "Done! \n" );
  return 0;
}
