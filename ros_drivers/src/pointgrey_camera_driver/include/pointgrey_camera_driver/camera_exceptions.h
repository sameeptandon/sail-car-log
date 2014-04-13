#ifndef _CAMERAEXCEPTIONS_H_
#define _CAMERAEXCEPTIONS_H_

#include <stdexcept>

class CameraTimeoutException: public std::runtime_error{
  public:
    CameraTimeoutException():runtime_error("Image not found within timeout."){}
    CameraTimeoutException(std::string msg):runtime_error(msg.c_str()){}
};

class CameraNotRunningException: public std::runtime_error{
  public:
    CameraNotRunningException():runtime_error("Camera is currently not running.  Please start the capture."){}
    CameraNotRunningException(std::string msg):runtime_error(msg.c_str()){}
};

class CameraImageNotReadyException: public std::runtime_error{
  public:
    CameraImageNotReadyException():runtime_error("Image is currently not ready."){}
    CameraImageNotReadyException(std::string msg):runtime_error(msg.c_str()){}
};

#endif
