#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Consumer_CV.h"

// FlyCapture SDK from Point Grey
#include "flycapture/FlyCapture2.h"

#define NUM_SPLITS 10

static const std::string OPENCV_WINDOW = "Image window";

class ThreadedImageWriter
{
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  SyncBuffer<cv::Mat> buffer[NUM_SPLITS];
  Consumer<cv::Mat>* consumer[NUM_SPLITS];
  
public:
    ThreadedImageWriter(std::string topic, ros::NodeHandle nh)
    : it_(nh)
  {

    for (int thread_num = 0; thread_num < NUM_SPLITS; thread_num++) {
        buffer[thread_num].getBuffer()->setCapacity(1000);
        consumer[thread_num] = new Consumer<cv::Mat>(
                buffer[thread_num].getBuffer(),
                "test_0.avi", buffer[thread_num].getMutex(),
                60.0f, 1280, 1024); 
    }
    ROS_INFO_STREAM("Subscribing to topic " << topic << "...");
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(topic, 1000, 
      &ThreadedImageWriter::imageCb, this);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ThreadedImageWriter()
  {
      for (int thread_num = 0; thread_num < NUM_SPLITS; thread_num++) { 
          consumer[thread_num]->stop();
          delete consumer[thread_num];
      }
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    
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

    cv::Mat* img = new cv::Mat( cv_ptr->image ) ;

    ROS_INFO_STREAM("buffer size:" << buffer[0].getBuffer()->getSize());

    if (!buffer[0].getBuffer()->pushBack(img)) {
        boost::mutex::scoped_lock( *(buffer[0].getMutex()));
        ROS_ERROR("Warning! Buffer full, overwriting data!");
    }

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(3);
    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_writer");
  ros::NodeHandle nh("~");
  std::string topic;
  nh.param<std::string>("image", topic, std::string(""));

  ROS_INFO_STREAM("Subscribing to topic " << topic << "...");
  ThreadedImageWriter ic(topic, nh);
  ros::spin();
  return 0;
}
