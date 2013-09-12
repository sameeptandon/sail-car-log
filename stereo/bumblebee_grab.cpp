#include <vector>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include "CameraLogger.h"
#include "bumblebee_xb3.h"
#include "VideoRecorder.h"

using namespace FlyCapture2;

void checkError(Error error) {
  if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
      exit(-1);
    }        
}

void ImageCallback(const Image& raw, SyncBuffer* camBuff) {
  Image* img = new Image;
  raw.Convert( PIXEL_FORMAT_BGR, img );
  if (!camBuff->getBuffer()->pushBack(img)) {
    boost::mutex::scoped_lock io_lock ( *(camBuff->getMutex()) );
    cerr << "Warning! Buffer full, overwriting data!" << endl;
  }
}

// Get current date/time, format is YYYY-MM-DD_HH:mm
const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d_%R", &tstruct);

  return buf;
}

int main( int argc, char* argv[] ) {
  std::string outdir="./recordings/";
  mkdir(outdir.c_str(), 0755);

  std::vector<std::string> names(3);
  names[0] = "left"; names[1] = "center"; names[2] = "right";

  float frameRate = 3.75f;
  unsigned int recordingSize = 500;
  
  // create and connect to camera
  BumblebeeXb3 cam;
  Error error;
  cam.format7AllCameras();
  cam.setFrameRate(frameRate);
  cam.StartCapture();

  const unsigned int camBuffSize = 3;
  SyncBuffer camBuff[camBuffSize];
  for (int c = 0; c < camBuffSize; c++)
    camBuff[c].getBuffer()->setCapacity(1000);

  std::vector<Consumer<Image>* > camConsumer(3);
  for (int c = 0; c < camBuffSize; c++)
    camConsumer[c] = new Consumer<Image>(camBuff[c].getBuffer(),
					 outdir+"/"+currentDateTime()+"_"+names[c]+".avi",
					 camBuff[c].getMutex(), frameRate, 1280, 960);
			      
  std::vector<Image> rawImgs(camBuffSize);
  for ( int imageCnt=0; imageCnt < recordingSize; imageCnt++ ) {
    // Retrieve an image
    cam.RetrieveBuffer( &rawImgs[0], &rawImgs[1], &rawImgs[2] );
    for (int i = 0; i < rawImgs.size(); i++) 
      ImageCallback(rawImgs[i], &camBuff[i]);
    printf( "Grabbed image %d\n", imageCnt );
  }

  for (int c = 0; c < camConsumer.size(); c++) 
    camConsumer[c]->stop();
  
  printf( "Done! \n" );
  return 0;
}
