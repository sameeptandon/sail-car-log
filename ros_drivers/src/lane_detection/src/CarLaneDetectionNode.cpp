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

caffe::Net<float> *lane_caffe_net;
caffe::Net<float> *car_caffe_net;
cv::Mat mean_image;
caffe::Blob<float> mean(1,3,480,640);

int framePeriod = 1;
int numFrames = 0;

cv::Scalar dist2color(double dist, double max_dist = 90.0){
  //given a distance and a maximum distance, gives a color code for the distance.
  //red being closest, green is mid-range, blue being furthest
  double alpha = (dist/max_dist);
  cv::Scalar color(0,0,0);
  if(alpha<0.5)
  {
    color[2] = 255*(1-alpha*2);
    color[1] = 255*alpha*2;
  }
  else
  {  
    double beta = alpha-0.5;
    color[1] = 255*(1-beta*2);
    color[0] = 255*beta*2;
  }
  return color;
}

void drawResults(cv::Mat& image, const float* pix_label, const float* reg_label, bool predict_depth, double scaling, int num_regression, int quad_height, int quad_width, int grid_dim){
  // draw ground truth and predictions on image.
  int grid_length = grid_dim*grid_dim;
  int label_height = quad_height*grid_dim;
  int label_width = quad_width*grid_dim;
  int img_width = image.cols;
  int img_height = image.rows;
  double thresh=0.4;
  // retrieve labels and predictions 
  for (int z=0; z<grid_length;++z){
    for (int qy = 0; qy < quad_height; ++qy) {
      for (int qx = 0; qx < quad_width; ++qx) {
        int dx = z%grid_dim;
        int dy = z/grid_dim;
        int x = qx*grid_dim+dx;
        int y = qy*grid_dim+dy;
        double label_prob = (double)(*(pix_label+((z*quad_height+qy)*quad_width+qx)));
        label_prob = 1. / (1. + exp(-label_prob));
        //std::cout<<label_prob<<" ";
        // draw pixel label/pred
        double x1 = x-0.5<0? 0:x-0.5;
        double y1 = y-0.5<0? 0:y-0.5;
        double w = scaling - (x<0.5? 0.5-x:0) - (x1+scaling>img_width? x1+scaling-img_width:0);
        double h = scaling - (y<0.5? 0.5-y:0) - (y1+scaling>img_height? y1+scaling-img_height:0);
        cv::Mat roi = image(cv::Rect(x1*scaling, y1*scaling, w, h));
        cv::Mat color(roi.size(), CV_8UC3, cv::Scalar(0, 255, 0)); 
        cv::addWeighted(color, label_prob, roi, 1.0 - label_prob , 0.0, roi); 
        if (label_prob > thresh) {

          // draw reg label/pred
          float x_adj = (qx*grid_dim + grid_dim / 2) * scaling;
          float y_adj = (qy*grid_dim + grid_dim / 2) * scaling;
          float x_min = *(reg_label+((z*quad_height+qy)*quad_width+qx))+x_adj;
          float y_min = *(reg_label+(((z+grid_length)*quad_height+qy)*quad_width+qx))+y_adj;
          float x_max = *(reg_label+(((z+grid_length*2)*quad_height+qy)*quad_width+qx))+x_adj;
          float y_max = *(reg_label+(((z+grid_length*3)*quad_height+qy)*quad_width+qx))+y_adj;
          cv::Point p1(x_min, y_min);
          cv::Point p2(x_max, y_max);
          cv::Scalar lineColor(100,100,200);
          if(predict_depth){
            float min_depth = *(reg_label+(((z+grid_length*4)*quad_height+qy)*quad_width+qx));
            float max_depth = *(reg_label+(((z+grid_length*5)*quad_height+qy)*quad_width+qx));
            lineColor = dist2color((min_depth+max_depth)/2.);
          }
          // draw label and predictions on image.
          cv::line(image,p1,p2,lineColor, 2);
        }
      }
    }
  }
}

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
  
  //Lane Detection
  const std::vector<caffe::Blob<float>*>& resultLane = lane_caffe_net->Forward(bottom_vec);

  caffe::Blob<float>* pix_blob = resultLane[1];
  caffe::Blob<float>* bb_blob = resultLane[0];
  
  bool predict_depth = true;
  int quad_height = pix_blob->height();
  int quad_width = pix_blob->width();
  int grid_dim=4;
  int grid_length = grid_dim*grid_dim;
  int label_height = quad_height*grid_dim;
  int label_width = quad_width*grid_dim;
  int num_regression= predict_depth ? 6:4;
  double scaling = 8.0; // ratio of image size to pix label size
  const float* pix_label = pix_blob->cpu_data();
  const float* reg_label = bb_blob->cpu_data();
  drawResults(frame, pix_label, reg_label, 
                      predict_depth, scaling, num_regression, 
                      quad_height, quad_width, grid_dim);
  

  //Car Detection
  const std::vector<caffe::Blob<float>*>& resultCar = car_caffe_net->Forward(bottom_vec);
  
  cv::Mat pixel_mask(60,80,CV_8UC3,cv::Scalar::all(0));

  caffe::Blob<float>* pixel_pred = resultCar[1];
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

  caffe::Blob<float>* bb_pred = resultCar[0];
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
  numFrames++;
}


int main ( int argc,char** argv )
{
  ros::init ( argc,argv,"CarLaneDetectionNode" );
  ros::NodeHandle nh_;
  cv::namedWindow(OPENCV_WINDOW);
  if ( argc>7 ) {
    time_t rawTime;
    struct tm * timeInfo;
    char buffer[80];
    std::time ( &rawTime );
    timeInfo = std::localtime ( &rawTime );
    std::strftime ( buffer,80,"%F-%H-%M-%S",timeInfo );
    std::string time ( buffer );
    std::string suffix = "_"+time+".avi";
    vidRec.open ( argv[7]+suffix,CV_FOURCC ( 'M','P','4','V' ),20,cv::Size ( 640,480 ) );
  }

  caffe::Caffe::SetDevice(0);
  caffe::Caffe::set_mode(caffe::Caffe::GPU);
  caffe::Caffe::set_phase(caffe::Caffe::TEST);
  lane_caffe_net = new caffe::Net<float>(argv[1]);
  lane_caffe_net->CopyTrainedLayersFrom(argv[2]);
  car_caffe_net = new caffe::Net<float>(argv[3]);
  car_caffe_net->CopyTrainedLayersFrom(argv[4]);

  mean_image = cv::imread(argv[5]);

  std::vector<cv::Mat> splitChannels_mean;
  cv::split(mean_image,splitChannels_mean);
  float *mean_data = mean.mutable_cpu_data();
  int frameSize = mean_image.cols*mean_image.rows;
  for(int i = 0;i<3;i++){
    std::copy(splitChannels_mean[i].data, splitChannels_mean[i].data+frameSize, (float*)mean_data+frameSize*i);
  }

  ros::Subscriber sub = nh_.subscribe ( argv[6],1,caffeCallback );
  delay_pub = nh_.advertise<lane_detection::Delay> ( "carlane_detection/Delay",100 );
  ros::spin();
  cv::destroyWindow ( OPENCV_WINDOW );
  delete lane_caffe_net;
  delete car_caffe_net;
  return 0;
}
