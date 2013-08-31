#include <vector>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include "FlyCapture2.h"
#include "stereo_xb3.h"

using namespace FlyCapture2;

int main(int /*argc*/, char** /*argv*/) {
  
  const int k_numImages = 5;
  
  PrintBuildInfo();

  Error error;
  BusManager busMgr;
  unsigned int numCameras;
  
  Camera cam;
  PGRGuid guid = getOnlyGuid();
  // Connect to a camera
  error = cam.Connect(&guid);
  if (error != PGRERROR_OK) {
      error.PrintErrorTrace();
      return -1;
  }

  // print info
  CameraInfo camInfo;
  error = cam.GetCameraInfo(&camInfo);
  if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
      return -1;
    }  
  PrintCameraInfo(&camInfo);

  // set video mode
  error = cam.SetVideoModeAndFrameRate( 
  				       VIDEOMODE_1280x960Y8, 
  				       FRAMERATE_1_875 );
  if (error != PGRERROR_OK){
      error.PrintErrorTrace();
      return -1;
  }

  // set configuration so we capture from all three cameras
  setXb3Configuration(&cam);

  // Start capturing images
  error = cam.StartCapture();
  if (error != PGRERROR_OK) {
    error.PrintErrorTrace();
    return -1;
  }

  Image rawImage;
  Image right, center, left;
  for ( int imageCnt=0; imageCnt < k_numImages; imageCnt++ ) {
    
    // Retrieve an image
      error = cam.RetrieveBuffer( &rawImage );
      if (error != PGRERROR_OK)
        {
	  error.PrintErrorTrace();
	  continue;
        }

      printf( "Grabbed image %d\n", imageCnt );

      // get other images
      deinterlace(rawImage, &right, &center, &left);

      // Save files
      std::stringstream ss;
      ss << imageCnt;
      std::string filename = ss.str();
      left.Save( ("left-"+filename+".ppm").c_str() );
      center.Save( ("center-"+filename+".ppm").c_str() );
      right.Save( ("right-"+filename+".ppm").c_str() );
  }
  
  // Stop capturing images
  error = cam.StopCapture();
  if (error != PGRERROR_OK) {
    error.PrintErrorTrace();
    return -1;
  }      

  // Disconnect the camera
  error = cam.Disconnect();
  if (error != PGRERROR_OK) {
    error.PrintErrorTrace();
    return -1;
  }

  printf( "Done! \n" );


  return 0;
}
