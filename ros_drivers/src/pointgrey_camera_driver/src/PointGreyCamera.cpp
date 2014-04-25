#include "pointgrey_camera_driver/PointGreyCamera.h"
#include <iostream>

using namespace FlyCapture2;
#define SHUTTER_PARAM (190) // 3 ms

PointGreyCamera::PointGreyCamera():
   busMgr_(), cam_(){
   serial_ = 0;
   captureRunning_ = false;
}

PointGreyCamera::~PointGreyCamera(){
}

void PointGreyCamera::writeRegister(uint32_t address, uint32_t value) { 
    cam_.WriteRegister(address, value);
}

uint32_t PointGreyCamera::readRegister(uint32_t address) { 
    uint32_t pValue;
    cam_.ReadRegister(address, &pValue);
    return pValue;
}

void PointGreyCamera::setWhiteBalance(int red, int blue) {
    Error error;
    Property prop;
    prop.type = WHITE_BALANCE;
    prop.absControl = false;
    prop.present = true;
    prop.onOff = false;
    prop.autoManualMode = false;
    prop.valueA = red;
    prop.valueB = blue;
    error = cam_.SetProperty(&prop);
    PointGreyCamera::handleError("set white balance fail", error);
}


void PointGreyCamera::setTimeout(const double &timeout){
  /*
  FC2Config pConfig;
  Error error = cam_.GetConfiguration(&pConfig);
  PointGreyCamera::handleError("PointGreyCamera::setTimeout Could not get camera configuration", error);
  pConfig.grabTimeout = (int)(1000.0*timeout); // Needs to be in ms
  if(pConfig.grabTimeout < 0.00001){
    pConfig.grabTimeout = -1; // Default - no timeout
  }
  error = cam_.SetConfiguration(&pConfig);
  PointGreyCamera::handleError("PointGreyCamera::setTimeout Could not set camera configuration", error);
  */
}

void PointGreyCamera::connect(){
    Error error;
    PGRGuid guid;  // GUIDS are NOT persistent accross executions, do not store them.
    error = busMgr_.GetCameraFromSerialNumber(serial_, &guid);
    std::stringstream serial_string;
    serial_string << serial_;
    std::string msg = "PointGreyCamera::connect Could not find camera with serial number: " + serial_string.str() + ". Is that camera plugged in?";
    PointGreyCamera::handleError(msg, error);
    error = cam_.Connect(&guid);
    PointGreyCamera::handleError("PointGreyCamera::connect Failed to connect to camera", error);
    
    // Enable metadata
    /*
    EmbeddedImageInfo info;
    info.timestamp.onOff = true;
    info.gain.onOff = true;
    info.shutter.onOff = true;
    info.brightness.onOff = true;
    info.exposure.onOff = true;
    info.whiteBalance.onOff = true;
    info.frameCounter.onOff = true;
    info.ROIPosition.onOff = true;
    error = cam_.SetEmbeddedImageInfo(&info);
    PointGreyCamera::handleError("PointGreyCamera::connect Could not enable metadata", error);
    */

    // set video frame rate and mode
    error = cam_.SetVideoModeAndFrameRate(VIDEOMODE_1280x960YUV422, FRAMERATE_60);
    PointGreyCamera::handleError("PointGreyCamera::connect could not enable framerate", error);

    // set shutter settings
    uint32_t bytes = readRegister(0x1098);
    bytes = bytes & (-1 << 12);
    bytes = bytes | SHUTTER_PARAM; 
    writeRegister(0x1098, bytes);

    // set white balance control
    setWhiteBalance(511, 815);

    // set databus speed
    FC2Config pConfig;
    cam_.GetConfiguration(&pConfig);
    pConfig.grabMode = BUFFER_FRAMES;
    pConfig.numBuffers = 100;
    //pConfig.isochBusSpeed = BUSSPEED_S5000;
    //pConfig.asyncBusSpeed = BUSSPEED_S5000;
    pConfig.grabTimeout = (int)(1000.0); // Needs to be in ms
    pConfig.highPerformanceRetrieveBuffer = true;
    cam_.SetConfiguration(&pConfig);

    //set external triggering
    TriggerMode mTrigger;
    mTrigger.mode = 0;
    mTrigger.source = 0;
    mTrigger.parameter = 0;
    mTrigger.onOff = true;
    mTrigger.polarity = 1;
    error = cam_.SetTriggerMode(&mTrigger); 
    PointGreyCamera::handleError("could not set trigger mode", error);

    using namespace std;
}

void PointGreyCamera::disconnect(){
  boost::mutex::scoped_lock scopedLock(mutex_);
  captureRunning_ = false;
  if(cam_.IsConnected()){
    Error error = cam_.Disconnect();
    PointGreyCamera::handleError("PointGreyCamera::disconnect Failed to disconnect camera", error);
  }
}

void PointGreyCamera::start(){
  if(cam_.IsConnected() && !captureRunning_){
    // Start capturing images
    Error error = cam_.StartCapture();
    PointGreyCamera::handleError("PointGreyCamera::start Failed to start capture", error);
    captureRunning_ = true;
  }
}

bool PointGreyCamera::stop(){
  if(cam_.IsConnected() && captureRunning_){
    // Stop capturing images
    captureRunning_ = false;
    Error error = cam_.StopCapture();
    PointGreyCamera::handleError("PointGreyCamera::stop Failed to stop capture", error);
    return true;
  }
  return false;
}

void PointGreyCamera::grabImage(sensor_msgs::Image &image, const std::string &frame_id){
  boost::mutex::scoped_lock scopedLock(mutex_);
  if(cam_.IsConnected() && captureRunning_){
    // Make a FlyCapture2::Image to hold the buffer returned by the camera.
    Image rawImage;
    // Retrieve an image
    Error error = cam_.RetrieveBuffer(&rawImage);
    PointGreyCamera::handleError("PointGreyCamera::grabImage Failed to retrieve buffer", error);
    //metadata_ = rawImage.GetMetadata();
    
    // Set header timestamp as embedded for now
    /*
    TimeStamp embeddedTime = rawImage.GetTimeStamp();
    image.header.stamp.sec = embeddedTime.seconds;
    image.header.stamp.nsec = 1000*embeddedTime.microSeconds;
    */
    image.header.stamp = ros::Time::now();
    
    // Get camera info to check if color or black and white chameleon and check the bits per pixel.
    CameraInfo cInfo;
    error = cam_.GetCameraInfo(&cInfo);
    PointGreyCamera::handleError("PointGreyCamera::grabImage  Failed to get camera info.", error);
    uint8_t bitsPerPixel = rawImage.GetBitsPerPixel();
    
    // Set the image encoding
    //std::string imageEncoding = sensor_msgs::image_encodings::MONO8;
    //if(cInfo.isColorCamera && rawImage.GetBayerTileFormat() != NONE){
	//    imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR8;
    //}

    //std::string imageEncoding = sensor_msgs::image_encodings::YUV422;
    //fillImage(image, imageEncoding, rawImage.GetRows(), rawImage.GetCols(), rawImage.GetStride(), rawImage.GetData());
    std::string imageEncoding = sensor_msgs::image_encodings::BGR8;
    Image bgrImage;
    rawImage.Convert(PIXEL_FORMAT_BGR, &bgrImage);
    fillImage(image, imageEncoding, bgrImage.GetRows(), bgrImage.GetCols(), bgrImage.GetStride(), bgrImage.GetData());
    image.header.frame_id = frame_id;
  } else if(cam_.IsConnected()){
    throw CameraNotRunningException("PointGreyCamera::grabImage: Camera is currently not running.  Please start the capture.");
  } else {
    throw std::runtime_error("PointGreyCamera::grabImage not connected!");
  }
}

void PointGreyCamera::setDesiredCamera(const uint32_t &id){
  serial_ = id;
}

std::vector<uint32_t> PointGreyCamera::getAttachedCameras(){
  std::vector<uint32_t> cameras;
  unsigned int num_cameras;
  Error error = busMgr_.GetNumOfCameras	(&num_cameras);
  PointGreyCamera::handleError("PointGreyCamera::getAttachedCameras: Could not get number of cameras", error);
  for(unsigned int i = 0; i < num_cameras; i++){
    unsigned int this_serial;
    error = busMgr_.GetCameraSerialNumberFromIndex(i, &this_serial);
    PointGreyCamera::handleError("PointGreyCamera::getAttachedCameras: Could not get get serial number from index", error);
    cameras.push_back(this_serial);
  }
  return cameras;
}

void PointGreyCamera::handleError(const std::string &prefix, FlyCapture2::Error &error) const{
    using namespace std;
    if (error == PGRERROR_TIMEOUT){
        throw CameraTimeoutException("PointGreyCamera: Failed to retrieve buffer within timeout.");
    }  else if (error != PGRERROR_OK){ // If there is actually an error (PGRERROR_OK means the function worked as intended...)
        std::string start(" | FlyCapture2::ErrorType ");
        std::stringstream out;
        out << error.GetType();
        std::string desc(error.GetDescription());
        throw std::runtime_error(prefix + start + out.str() + desc);
    }
}
