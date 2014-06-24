#include "flycapture/FlyCapture2.h"
#include "ros/ros.h"
#include "CameraHelper.h"
#include <image_transport/image_transport.h> // ROS library that allows sending compressed images
#include <camera_info_manager/camera_info_manager.h> // ROS library that publishes CameraInfo topics
#include <sensor_msgs/CameraInfo.h> // ROS message header for CameraInfo
#include <sensor_msgs/Image.h> // ROS message header for Image
#include <sensor_msgs/image_encodings.h> // ROS header for the different supported image encoding types
#include <sensor_msgs/fill_image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace FlyCapture2;
using namespace std;

void convertToCV(const Image* obj, IplImage* img) {
    img->height = obj->GetRows();
    img->width = obj->GetCols();
    img->widthStep = obj->GetStride();
    img->nChannels = 3;
    img->imageData = (char*) obj->GetData(); 
}


int main(int argc, char **argv){
    ros::init(argc, argv, "pointgrey_camera_driver");
    ros::NodeHandle nh("~");

    // Get the serial number of the camera to connect to
    int serial;
    nh.param<int>("serial", serial, 0);

    // Try connecting to that camera
    Camera* cam = ConnectCamera(serial);
    if (cam == NULL) {
        std::stringstream serial_string;
        serial_string << serial;
        std::string msg = "PointGreyCamera::connect Could not find camera with serial number: " + serial_string.str() + ". Is that camera plugged in?";
        throw std::runtime_error(msg);
    }

    cout << "connection success!" << endl;

    // What type of camera is it? Wide angle or narrow angle?
    string camera_type;
    nh.param<string>("camera_type", camera_type, "");

    // Connection successful! lets setup the ros topic. 
    
    // Get the location of our camera config yaml
    string camera_info_url, frame_id;
    nh.param<std::string>("camera_info_url", camera_info_url, "");
    // Get the desired frame_id, set to 'camera' if not found
    nh.param<std::string>("frame_id", frame_id, "camera");
   
    // Start the camera info manager and attempt to load any configurations
    std::stringstream cinfo_name;
    cinfo_name << serial;
    camera_info_manager::CameraInfoManager* cinfo = new camera_info_manager::CameraInfoManager(nh, cinfo_name.str(), camera_info_url);
    
    // Publish topics using ImageTransport through camera_info_manager (gives cool things like compression)
    image_transport::ImageTransport* it = new image_transport::ImageTransport(nh);
    image_transport::CameraPublisher it_pub = it->advertiseCamera("/" + ros::this_node::getNamespace() + "/image_raw", 1000);

    // get ready for capture loop
    Image image;
    Image bgrImage;
    IplImage* cvImg;
    string imageEncoding = sensor_msgs::image_encodings::BGR8;
    sensor_msgs::ImagePtr ros_image; ///< Camera Info message.
    sensor_msgs::CameraInfoPtr ci; ///< Camera Info message.

    uint64_t num_frames = 0; 
    
    // Now configure the camera and put it in run mode

    if (camera_type.compare("narrow") == 0)
        RunCamera(cam);
    else if (camera_type.compare("wide") == 0)
        RunWideAngleCamera(cam);
    else throw std::runtime_error("Camera type not specified");

    // capture loop
    while (ros::ok()) {

        // try to get an image
        Error error = cam->RetrieveBuffer(&image);

        //check if we had a timeout or some other error 
        if (error == PGRERROR_TIMEOUT) 
            continue;
        else if (error != PGRERROR_OK)
            throw std::runtime_error(error.GetDescription());

        // HACK: throw away half the frames for the WFOV cameras
        num_frames++;
        if (camera_type.compare("wide") == 0 && num_frames % 2 != 1)
            continue;

        //grabbed image, reset ros structures and fill out fields
        ros_image.reset(new sensor_msgs::Image());
        ci.reset(new sensor_msgs::CameraInfo(cinfo->getCameraInfo()));

        ros_image->header.stamp = ros::Time::now();
        ros_image->header.frame_id = frame_id;
        ci->header.stamp = ros_image->header.stamp;
        ci->header.frame_id = frame_id;
        
        //convert to bgr
        image.Convert(PIXEL_FORMAT_BGR, &bgrImage);
        cvImg = cvCreateImageHeader(cvSize(bgrImage.GetCols(), bgrImage.GetRows()), IPL_DEPTH_8U, 3);
        convertToCV(&bgrImage, cvImg);

        if (camera_type.compare("wide") == 0)
            cvFlip(cvImg, cvImg, -1);

        // package in ros header 
         //fillImage(*ros_image, imageEncoding, bgrImage.GetRows(), bgrImage.GetCols(), bgrImage.GetStride(), bgrImage.GetData());
        fillImage(*ros_image, imageEncoding, cvImg->height, cvImg->width, cvImg->widthStep, cvImg->imageData);

        // Publish the message using standard image transport
        it_pub.publish(ros_image, ci);
    }

    // node asked to terminate
    CloseCamera(cam);
    delete cam; 
}
