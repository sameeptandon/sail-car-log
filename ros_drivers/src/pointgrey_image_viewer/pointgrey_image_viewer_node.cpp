#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// FlyCapture SDK from Point Grey
#include "flycapture/FlyCapture2.h"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  
public:
  ImageConverter(std::string topic, ros::NodeHandle nh)
    : it_(nh)
  {
    ROS_INFO_STREAM("Subscribing to topic " << topic << "...");
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(topic, 1000, 
      &ImageConverter::imageCb, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
      FlyCapture2::Image z;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


    cv::Mat img = cv_ptr->image; 
    cv::pyrDown(img, img, cv::Size(img.cols/2, img.rows/2));
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(3);
    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_writer");
  ros::NodeHandle nh("~");
  std::string topic;
  nh.param<std::string>("image", topic, std::string(""));

  ROS_INFO_STREAM("Subscribing to topic " << topic << "...");
  ImageConverter ic(topic, nh);
  ros::spin();
  return 0;
}
