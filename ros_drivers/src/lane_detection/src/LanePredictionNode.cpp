#include "FastCpp.h"
#include "io/ArrayIO.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "LanePredictor.h"
#include <vector>
#include <queue>

static const std::string OPENCV_WINDOW = "Image window";
LanePredictor* predictor;
std::queue<std::vector<float> > mData;

void push_data(const std::vector<float>& data){
        mData.push(data);
}

void lanePredictorCb(const sensor_msgs::ImageConstPtr& msg){
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

	cv::Mat frame = cv_ptr->image;
    
	int frameSize = frame.cols*frame.rows*frame.channels();
    std::vector<float> img;
	img.resize(frameSize);
	std::copy(frame.data,frame.data+frameSize,img.data());
    push_data(img);
    
	Ptr<ArrayViewHandle> arr = hostArrayAllocRM(DataType::FLOAT,DDim(3,640,480,1),0);
	synchronizeStream(0);
   
	ASSERT(arr->memoryHandle()->ptr());
	std::copy(mData.front().begin(),mData.front().end(), (float*)arr->memoryHandle()->ptr());
    mData.pop();
    
	std::vector<int> perm(4,0);
	perm[0] = 3;
	perm[1] = 0;
	perm[2] = 1;
	perm[3] = 2;

	Ptr<ArrayViewHandle> pred = predictor->processImage( permute(arr,perm) );
	int num_classifier = pred->dim(1);
	float pixel_pred[num_classifier];
	std::copy((float*)pred->memoryHandle()->ptr(),(float*)pred->memoryHandle()->ptr()+num_classifier,pixel_pred);
	for(int i=0;i<24;i++){
		cv::circle(frame,cv::Point(pixel_pred[i]*8.0,pixel_pred[i+24]*4.0+160.0),2,cv::Scalar(0,0,255),-1,8);
	}
	// Update GUI Window
	cv::imshow(OPENCV_WINDOW, frame);
	cv::waitKey(3);
}


int main(int argc,char** argv){
	//fastCppInit(&argc,&argv);
	ros::init(argc,argv,"LanePredictorNode");
	ros::NodeHandle nh_;
	cv::namedWindow(OPENCV_WINDOW);
	predictor = new LanePredictor(&argc,argv,0);
	//LanePredictor predictor(&argc,argv,0);
	ros::Subscriber sub = nh_.subscribe("VideoProducer/output_video",1,lanePredictorCb);
	ros::spin();
	cv::destroyWindow(OPENCV_WINDOW);
	//fastCppShutdown();
	return 0;
}
