// ROS and associated nodelet interface and PLUGINLIB declaration header
#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "pointgrey_camera_driver/PointGreyCamera.h" // The actual standalone library for the PointGreys

#include <image_transport/image_transport.h> // ROS library that allows sending compressed images
#include <camera_info_manager/camera_info_manager.h> // ROS library that publishes CameraInfo topics
#include <sensor_msgs/CameraInfo.h> // ROS message header for CameraInfo

#include <boost/thread.hpp> // Needed for the nodelet to launch the reading thread.


namespace pointgrey_camera_driver{

class PointGreyCameraNodelet: public nodelet::Nodelet
{
public:
  PointGreyCameraNodelet(){
  init_ = false;
  }

  ~PointGreyCameraNodelet(){
    if(pubThread_){
      pubThread_->interrupt();
      pubThread_->join();
    }
    
    try{
      NODELET_DEBUG("Stopping camera capture.");
      pg_.stop();
      NODELET_DEBUG("Disconnecting from camera.");
      pg_.disconnect();
    } catch(std::runtime_error& e){
      NODELET_ERROR("%s", e.what());
    }
  }

private:
  
  /*!
  * \brief Connection callback to only do work when someone is listening.
  *
  * This function will connect/disconnect from the camera depending on who is using the output.
  */
  void connectCb(){
    NODELET_DEBUG("Connect callback!");
    
    /*
    boost::mutex::scoped_lock scopedLock(connect_mutex_); // Grab the mutex.  Wait until we're done initializing before letting this function through.
    // Check if we should disconnect (there are 0 subscribers to our data)
    if(it_pub_.getNumSubscribers() == 0){
        try{
            NODELET_DEBUG("Disconnecting.");
            pubThread_->interrupt();
            pubThread_->join();
            init_ = false;
            NODELET_DEBUG("Stopping camera capture.");
            pg_.stop();
        } catch(std::runtime_error& e){
            NODELET_ERROR("%s", e.what());
        }
    } else*/ if(!init_) { // We need to connect
        NODELET_DEBUG("Connecting");
        // Try connecting to the camera
        volatile bool connected = false;
        while(!connected && ros::ok()){
            try{
                NODELET_DEBUG("Connecting to camera.");
                pg_.connect(); // Probably already connected from the reconfigure thread.  This will will not throw if successfully connected.
                connected = true;
            } catch(std::runtime_error& e){
                NODELET_ERROR("%s", e.what());
                ros::Duration(1.0).sleep(); // sleep for one second each time
            }
        }

        // Set the timeout for grabbing images.
        double timeout;
        getMTPrivateNodeHandle().param("timeout", timeout, 1.0);
        try{
            NODELET_DEBUG("Setting timeout to: %f.", timeout);
            pg_.setTimeout(timeout);
        } catch(std::runtime_error& e){
            NODELET_ERROR("%s", e.what());
        }

        // Subscribe to gain and white balance changes
        init_ = true;

        volatile bool started = false;
        while(!started && ros::ok()){
            try{
                NODELET_DEBUG("Starting camera capture.");
                pg_.start();
                started = true;
            } catch(std::runtime_error& e){
                NODELET_ERROR("%s", e.what());
                ros::Duration(1.0).sleep(); // sleep for one second each time
            }
        }

        // Start the thread to loop through and publish messages
        pubThread_.reset(new boost::thread(boost::bind(&pointgrey_camera_driver::PointGreyCameraNodelet::devicePoll, this)));
    } else {
        NODELET_DEBUG("Do nothing in callback.");
    }
  }

  /*!
   * \brief Serves as a psuedo constructor for nodelets.
   *
  * This function needs to do the MINIMUM amount of work to get the nodelet running.  Nodelets should not call blocking functions here.
  */
  void onInit(){
    // Get nodeHandles
    ros::NodeHandle &nh = getMTNodeHandle();
    ros::NodeHandle &pnh = getMTPrivateNodeHandle();
    
    // Get a serial number through ros
    int serial;
    pnh.param<int>("serial", serial, 0);
    pg_.setDesiredCamera((uint32_t)serial);

    // Get the location of our camera config yaml
    std::string camera_info_url;
    pnh.param<std::string>("camera_info_url", camera_info_url, "");
    // Get the desired frame_id, set to 'camera' if not found
    pnh.param<std::string>("frame_id", frame_id_, "camera");
    num_frames_ = 0; 
        
    // Do not call the connectCb function until after we are done initializing.
    //boost::mutex::scoped_lock scopedLock(connect_mutex_);
   
    // Start the camera info manager and attempt to load any configurations
    std::stringstream cinfo_name;
    cinfo_name << serial;
    cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, cinfo_name.str(), camera_info_url));
    
    // Publish topics using ImageTransport through camera_info_manager (gives cool things like compression)
    it_.reset(new image_transport::ImageTransport(nh));
    image_transport::SubscriberStatusCallback cb = boost::bind(&PointGreyCameraNodelet::connectCb, this);
    it_pub_ = it_->advertiseCamera("image_raw", 1000, cb, cb);
    //connectCb();
  }
  
  /*!
  * \brief Function for the boost::thread to grabImages and publish them.
  *
  * This function continues until the thread is interupted.  Responsible for getting sensor_msgs::Image and publishing them.
  */
  void devicePoll(){
    while (!boost::this_thread::interruption_requested())  // Block until we need to stop this thread.
    {
        image_.reset(new sensor_msgs::Image());

        // Get the image from the camera library
        NODELET_DEBUG("Starting a new grab from camera.");
        using namespace std;
        try { 
            pg_.grabImage(*image_, frame_id_);
        } catch (CameraTimeoutException &e) {
            continue;
        }

        // Set the CameraInfo message
        ci_.reset(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
        ci_->header.stamp = ros::Time::now();
        ci_->header.frame_id = frame_id_;

        // Publish the message using standard image transport
        //if(it_pub_.getNumSubscribers() > 0){
            it_pub_.publish(image_, ci_);
            num_frames_+= 1;
        //}

    }
  }

  bool init_;
  
  boost::shared_ptr<image_transport::ImageTransport> it_; ///< Needed to initialize and keep the ImageTransport in scope.
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_; ///< Needed to initialize and keep the CameraInfoManager in scope.
  image_transport::CameraPublisher it_pub_; ///< CameraInfoManager ROS publisher
  //boost::shared_ptr<diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage> > pub_; ///< Diagnosed publisher, has to be a pointer because of constructor requirements
  
  boost::mutex connect_mutex_;
  
  PointGreyCamera pg_; ///< Instance of the PointGreyCamera library, used to interface with the hardware.
  sensor_msgs::CameraInfoPtr ci_; ///< Camera Info message.
  sensor_msgs::ImagePtr image_; ///< Camera Info message.
  std::string frame_id_; ///< Frame id for the camera messages, defaults to 'camera'
  boost::shared_ptr<boost::thread> pubThread_; ///< The thread that reads and publishes the images.

  uint64_t num_frames_; 
  
};

PLUGINLIB_DECLARE_CLASS(pointgrey_camera_driver, PointGreyCameraNodelet, pointgrey_camera_driver::PointGreyCameraNodelet, nodelet::Nodelet);  // Needed for Nodelet declaration
}
