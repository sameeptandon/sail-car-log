#include <vector>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

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

void ImageCallback(const Image& rawLeft, const Image& rawCenter, const Image& rawRight,
		   SyncBuffer camBuff[]) {
  std::vector<Image*> imgs(3);
  for (int i = 0; i < imgs.size(); i++)
    imgs[i] = new Image();
  //Image* im = new Image();
  //pImage->Convert(PIXEL_FORMAT_BGR, im);

  // rawRight.Convert( PIXEL_FORMAT_RGB8, imgs[0] );
  // rawCenter.Convert( PIXEL_FORMAT_RGB8, imgs[1] );
  // rawLeft.Convert( PIXEL_FORMAT_RGB8, imgs[2] );

  rawRight.Convert( PIXEL_FORMAT_BGR, imgs[0] );
  rawCenter.Convert( PIXEL_FORMAT_BGR, imgs[1] );
  rawLeft.Convert( PIXEL_FORMAT_BGR, imgs[2] );  

  for (int c = 0; c < 3; c++) {
    if (!camBuff[c].getBuffer()->pushBack(imgs[c])) {
      boost::mutex::scoped_lock io_lock ( *(camBuff[c].getMutex()) );
      cerr << "Warning! Buffer full, overwriting data!" << endl; 
    }
  }
}

int main( int argc, char* argv[] ) {
  std::string outdir="./images/";
  mkdir(outdir.c_str(),755);

  std::vector<std::string> names(3);
  names[0] = "left"; names[1] = "center"; names[2] = "right";

  float frameRate = 3.75f;
  unsigned int recordingSize = 500;
  
  // create and connect to camera
  BumblebeeXb3 cam;
  Error error;
  cam.format7AllCameras();
  cam.setFrameRate(frameRate);
  
  // checkError( cam.camera().SetVideoModeAndFrameRate( VIDEOMODE_1280x960Y8, 
  // 						     FRAMERATE_3_75 ) );

  // create a video saving stream that runs on different thread
  //VideoSaver vid(framerate, format);
  
  cam.StartCapture();

  const unsigned int camBuffSize = 3;
  SyncBuffer camBuff[camBuffSize];
  //std::vector<SyncBuffer> camBuff(3);
  for (int c = 0; c < camBuffSize; c++)
    camBuff[c].getBuffer()->setCapacity(1000);

  std::vector<Consumer<Image>* > camConsumer(3);
  for (int c = 0; c < camBuffSize; c++)
    camConsumer[c] = new Consumer<Image>(camBuff[c].getBuffer(), "video_"+names[c]+".avi",
					 camBuff[c].getMutex(), frameRate, 1280, 960);
			      
  Image rawImage, rawRight, rawCenter, rawLeft, right, center, left;
  for ( int imageCnt=0; imageCnt < recordingSize; imageCnt++ ) {
    // Retrieve an image
    cam.RetrieveBuffer( &rawLeft, &rawCenter, &rawRight );
    //cam.RetrieveBuffer( &rawImage );
    ImageCallback(rawLeft, rawCenter, rawRight, camBuff);
    //vid.record( rawLeft );
    
    // if (imageCnt % 100 == 0)
    //   //vid.save(filename);
    
    // rawRight.Convert( PIXEL_FORMAT_RGB8, &right );
    // rawCenter.Convert( PIXEL_FORMAT_RGB8, &center );
    // rawLeft.Convert( PIXEL_FORMAT_RGB8, &left );
    
    printf( "Grabbed image %d\n", imageCnt );

    // Save files
    // std::stringstream ss;
    // ss << imageCnt;
    // std::string filename = ss.str();
    // checkError( left.Save( (outdir+"/"+filename+"_left.png").c_str() ) );
    // checkError( center.Save( (outdir+"/"+filename+"_center.png").c_str() ) );
    // checkError( right.Save( (outdir+"/"+filename+"_right.png").c_str() ) );
  }
  
  //vid.save( filename );
  //vid.finish();
  for (int c = 0; c < camConsumer.size(); c++) 
    camConsumer[c]->stop();
  
  printf( "Done! \n" );
  return 0;
}
