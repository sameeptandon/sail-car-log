#include "io/ArrayIO.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <queue>
#include "lane_detection/Delay.h"
#include "lane_detection/LaneOutput.h"
#include <time.h>

#include "caffe/caffe.hpp"

// A simple registry for caffe commands.
typedef int (*BrewFunction)();
typedef std::map<caffe::string, BrewFunction> BrewMap;
BrewMap g_brew_map;

#define RegisterBrewFunction(func) \
namespace { \
class __Registerer_##func { \
 public: /* NOLINT */ \
  __Registerer_##func() { \
    g_brew_map[#func] = &func; \
  } \
}; \
__Registerer_##func g_registerer_##func; \
}

static const std::string OPENCV_WINDOW = "Image window";
ros::Publisher delay_pub;
float invKK[9] = {  4.43506182e-04f, 0.0f, -2.90740478e-01f,
                    0.0f, 4.41247849e-04f, -2.15704011e-01f,
                    0.0f,0.0f,1.0f
                 };
float R[9] = {  0.99951053f,-0.01020889f,0.02957164f,
                0.01149488f,0.99898081f,-0.04364862f,
                -0.02909589f,0.04396718f,0.99860919f
             };
float leftPixels[30];
float rightPixels[30];
boost::array<float,30> leftPos;
boost::array<float,30> rightPos;
float zPos[10] = {12.0,20.0,28.0,36.0,44.0,52.0,60.0,68.0,76.0,84.0};

std::vector<tf::Vector3> leftPosVec ( 10 );
std::vector<tf::Vector3> rightPosVec ( 10 );
tf::Quaternion camToVehRot ( -0.0238,-0.06237513,0.0049506 );
tf::Vector3 camToVehTrans ( 0,0,-0.049506 );
tf::Transform camToVeh ( camToVehRot,camToVehTrans );

cv::VideoWriter vidRec;

caffe::Net<float> *caffe_net;
cv::Mat mean_image;
caffe::Blob<float> mean(1,3,480,640);

int framePeriod = 1;


void caffeCallback(const sensor_msgs::ImageConstPtr& msg){
  lane_detection::Delay delay_msg;
  delay_msg.cam_frame = msg->header.stamp;
  delay_msg.start_proc = ros::Time::now();

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare ( msg, sensor_msgs::image_encodings::BGR8 );
  } catch ( cv_bridge::Exception& e ) {
    ROS_ERROR ( "cv_bridge exception: %s", e.what() );
    return;
  }

  cv::Mat frame = cv_ptr->image;
  if ( frame.cols==1280 ) {
    cv::pyrDown ( frame, frame, cv::Size ( frame.cols/2, frame.rows/2 ) );
  }
  std::vector<cv::Mat> splitChannels;
  cv::split(frame,splitChannels);
  int frameSize = frame.cols*frame.rows;

  caffe::Blob<float> input(1,3,480,640);
  float *input_data = input.mutable_cpu_data();
  for(int i = 0;i<3;i++){
    std::copy(splitChannels[i].data, splitChannels[i].data+frameSize, (float*)input_data+frameSize*i);
  }
  float *mean_data = mean.mutable_cpu_data();
  for(int i =0;i<3*480*640;i++){
    input_data[i] -= mean_data[i];
  }

  std::vector<caffe::Blob<float>* > bottom_vec;
  bottom_vec.push_back(&input);
  const std::vector<caffe::Blob<float>*>& result =caffe_net->Forward(bottom_vec);

  cv::Mat pixel_mask(60,80,CV_8UC3,cv::Scalar::all(0));

  caffe::Blob<float>* pixel_pred = result[1];
  pixel_pred->Reshape(4,4,15,20);
  for(int i=0;i<20;i++){
    for(int j=0;j<15;j++){
      for(int k=0;k<4;k++){
        for(int m=0;m<4;m++){
          cv::Vec3b &color = pixel_mask.at<cv::Vec3b>(j*4+m,i*4+k);
          color[2] = pixel_pred->data_at(m,k,j,i)*255;
        }
      }
    }
  }

  cv::Mat hard_mask;
  cv::threshold(pixel_mask,hard_mask,0.25*255,255,CV_THRESH_BINARY);
  cv::Mat upscaled_pixel_mask;
  cv::resize( pixel_mask, upscaled_pixel_mask, cv::Size(pixel_mask.cols*8,pixel_mask.rows*8));

  frame += upscaled_pixel_mask;

  caffe::Blob<float>* bb_pred = result[0];
  bb_pred->Reshape(4,16,15,20);
  std::vector<cv::Rect> rects;
  float bb_pts[4];
  for(int i=0;i<20;i++){
    int x_offset = i*32+16;
    for(int j=0;j<15;j++){
      int y_offset = j*32+16;
      for(int k=0;k<4;k++){
        for(int m=0;m<4;m++){
          bb_pts[0] = bb_pred->data_at(0,k+m*4,j,i)+x_offset;
          bb_pts[1] = bb_pred->data_at(1,k+m*4,j,i)+y_offset;
          bb_pts[2] = bb_pred->data_at(2,k+m*4,j,i)+x_offset;
          bb_pts[3] = bb_pred->data_at(3,k+m*4,j,i)+y_offset;
          if(hard_mask.at<cv::Vec3b>(j*4+m,i*4+k)[2] > 0 && bb_pts[2]-bb_pts[0] > 0 && bb_pts[3]-bb_pts[1] > 0){
            bb_pts[0] = std::min(640.0f,std::max(0.0f,bb_pts[0]));
            bb_pts[2] = std::min(640.0f,std::max(0.0f,bb_pts[2]));
            bb_pts[1] = std::min(480.0f,std::max(0.0f,bb_pts[1]));
            bb_pts[3] = std::min(480.0f,std::max(0.0f,bb_pts[3]));
            rects.push_back(cv::Rect(cv::Point(bb_pts[0],bb_pts[1]),cv::Point(bb_pts[2],bb_pts[3])));
          }
        }
      }
    }
  }

  cv::groupRectangles(rects,4,0.4);
  for(int i=0;i<rects.size();i++){
    cv::rectangle(frame,rects[i],cv::Scalar(0,255,0));
  }

  delay_msg.end_proc = ros::Time::now();
  delay_msg.proc_time = delay_msg.end_proc-delay_msg.start_proc;
  delay_msg.cam_frame_delay = delay_msg.end_proc-delay_msg.cam_frame;
  delay_pub.publish ( delay_msg );

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, frame);
  cv::waitKey ( 3 );
}


int main ( int argc,char** argv )
{
  ros::init ( argc,argv,"CarDetectionNode" );
  ros::NodeHandle nh_;
  cv::namedWindow(OPENCV_WINDOW);
  if ( argc>5 ) {
    time_t rawTime;
    struct tm * timeInfo;
    char buffer[80];
    std::time ( &rawTime );
    timeInfo = std::localtime ( &rawTime );
    std::strftime ( buffer,80,"%F-%H-%M-%S",timeInfo );
    std::string time ( buffer );
    std::string suffix = "_"+time+".avi";
    vidRec.open ( argv[5]+suffix,CV_FOURCC ( 'M','P','4','V' ),20,cv::Size ( 640,480 ) );
  }

  caffe::Caffe::SetDevice(0);
  caffe::Caffe::set_mode(caffe::Caffe::GPU);
  caffe::Caffe::set_phase(caffe::Caffe::TEST);
  caffe_net = new caffe::Net<float>(argv[1]);
  caffe_net->CopyTrainedLayersFrom(argv[2]);
  mean_image = cv::imread(argv[3]);

  std::vector<cv::Mat> splitChannels_mean;
  cv::split(mean_image,splitChannels_mean);
  float *mean_data = mean.mutable_cpu_data();
  int frameSize = mean_image.cols*mean_image.rows;
  for(int i = 0;i<3;i++){
    std::copy(splitChannels_mean[i].data, splitChannels_mean[i].data+frameSize, (float*)mean_data+frameSize*i);
  }

  ros::Subscriber sub = nh_.subscribe ( argv[4],1,caffeCallback );
  delay_pub = nh_.advertise<lane_detection::Delay> ( "car_detection/Delay",100 );
  ros::spin();
  cv::destroyWindow ( OPENCV_WINDOW );
  delete caffe_net;
  return 0;
}
