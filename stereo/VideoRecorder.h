#include <FlyCapture2.h>
#include <vector>
#include <string>

#pragma once


using namespace FlyCapture2;

enum AviType {
    UNCOMPRESSED,
    MJPG,
    H264
};


class VideoRecorder {

public:
  VideoRecorder(float framerate, AviType aviType) : mFrameRate(framerate), mAviType(aviType),
						    mThread(&VideoRecorder::writeToDisk, this),
						    mDone(false), mRecordings(&mIoMutex) {
    mCurrentRecording = new std::vector<Image*>;
  }

  
  void record( const Image& img ) {
    Image* imgPtr = new Image;
    imgPtr->DeepCopy(&img);
    mCurrentRecording->push_back(imgPtr);
  }

  void save( const std::string& filename) {
    // save to buffer so other thread reads it
    Recording* rec = new Recording;
    rec->imgs = mCurrentRecording;
    rec->name = filename;
    mRecordings.pushBack(rec);
    
    // start new recording
    mCurrentRecording = new std::vector<Image*>;
  }

  void writeToDisk() {
    while (!mDone) {
      const Recording* rec = mRecordings.getFront();
      AVIRecorder aviRecorder;
      Error error;
      AVIOpen(&aviRecorder, rec->name, rec->imgs->at(0)->GetRows(), rec->imgs->at(0)->GetCols());
      for (int i = 0; i < rec->imgs->size(); i++) {
	Image* img = *rec->imgs->data()+i;
	error = aviRecorder.AVIAppend(img);
        if (error != PGRERROR_OK) {
	    error.PrintErrorTrace();
	    continue;
	}
	delete img;
      }
      
      error = aviRecorder.AVIClose();
      if (error != PGRERROR_OK) {
	error.PrintErrorTrace();
	exit(-1);
      }
      delete rec;
    }
  }
  
  void stop() {
    mRecordings.waitUntilEmpty();
    mRecordings.setDone();
  }

  void AVIOpen( AVIRecorder* aviRecorder, const std::string& filename,
		unsigned int height, unsigned int width) {
    Error error;
    switch (mAviType)  {
    case UNCOMPRESSED:  {
      AVIOption option;
      option.frameRate = mFrameRate;
      error = aviRecorder->AVIOpen(filename.c_str(), &option);
    }
      break;
    case MJPG: {
      MJPGOption option;
      option.frameRate = mFrameRate;
      option.quality = 75;
      error = aviRecorder->AVIOpen(filename.c_str(), &option);
    }
      break;
    case H264: {
      H264Option option;
      option.frameRate = mFrameRate;
      option.bitrate = 1000000;
      option.height = height;
      option.width = width;
      error = aviRecorder->AVIOpen(filename.c_str(), &option);
    }
      break;
    }

    if (error != PGRERROR_OK) {
      error.PrintErrorTrace();
      exit(-1);
    }
  }  

private:

  struct Recording {
    ~Recording() {
      if (imgs) delete imgs;
    }
    std::string name;    
    std::vector<Image*>* imgs;
  };
  
  SynchronizedBuffer<Recording> mRecordings;
  std::vector<Image*>* mCurrentRecording;
  AviType mAviType;
  float mFrameRate;
  bool mDone;
  boost::mutex mIoMutex;
  boost::thread mThread;
};
