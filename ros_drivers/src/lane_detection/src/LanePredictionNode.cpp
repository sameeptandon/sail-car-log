#include "FastCpp.h"
#include "io/ArrayIO.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "LanePredictor_Q50.h"
#include <vector>
#include <queue>
#include "lane_detection/Delay.h"
#include "lane_detection/LaneOutput.h"
#include <time.h>

static const std::string OPENCV_WINDOW = "Image window";
LanePredictor* predictor;
std::queue<std::vector<float> > mData;
ros::Publisher delay_pub,laneOutput_pub;
float invKK[9] = {  4.43506182e-04f, 0.0f, -2.90740478e-01f,
                    0.0f, 4.41247849e-04f, -2.15704011e-01f,
                    0.0f,0.0f,1.0f};
float R[9] = {  0.99951053f,-0.01020889f,0.02957164f,
                0.01149488f,0.99898081f,-0.04364862f,
                -0.02909589f,0.04396718f,0.99860919f};
float leftPixels[30];
float rightPixels[30];
boost::array<float,30> leftPos;
boost::array<float,30> rightPos;
float zPos[10] = {12.0,20.0,28.0,36.0,44.0,52.0,60.0,68.0,76.0,84.0};

std::vector<tf::Vector3> leftPosVec(10);
std::vector<tf::Vector3> rightPosVec(10);
tf::Quaternion camToVehRot(-0.0238,-0.06237513,0.0049506);
tf::Vector3 camToVehTrans(0,0,-0.049506);
tf::Transform camToVeh(camToVehRot,camToVehTrans);

Ptr<ArrayViewHandle> arr;
cv::VideoWriter vidRec;

void push_data(const std::vector<float>& data){
        mData.push(data);
}

int framePeriod = 3;

void lanePredictorCb(const sensor_msgs::ImageConstPtr& msg){
    if(msg->header.seq%framePeriod!=0){
        return;
    }
    lane_detection::Delay delay_msg;
    lane_detection::LaneOutput lane_msg;
    delay_msg.cam_frame = msg->header.stamp;
    //delay_pub.publish(delay_msg);
    //return;
    cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

    //ROS_INFO("%u.%u",msg->header.stamp.sec,msg->header.stamp.nsec);
	cv::Mat frame = cv_ptr->image;
    if(frame.cols==1280){
        cv::pyrDown(frame, frame, cv::Size(frame.cols/2, frame.rows/2));
    }
	int frameSize = frame.cols*frame.rows*frame.channels();
    //std::vector<float> img;
	//img.resize(frameSize);
	//std::copy(frame.data,frame.data+frameSize,img.data());
    //push_data(img);
    
	   
	ASSERT(arr->memoryHandle()->ptr());
	//std::copy(mData.front().begin(),mData.front().end(), (float*)arr->memoryHandle()->ptr());
    std::copy(frame.data,frame.data+frameSize,(float*)arr->memoryHandle()->ptr());
    //mData.pop();
    //arr->memoryHandle()->setPtr(frame.data,sizeof(float)*frameSize);

	std::vector<int> perm(4,0);
	perm[0] = 3;
	perm[1] = 0;
	perm[2] = 1;
	perm[3] = 2;
	delay_msg.start_proc = ros::Time::now();
	Ptr<ArrayViewHandle> pred = predictor->processImage( permute(arr,perm) );
    synchronizeStream(0);
	delay_msg.end_proc = ros::Time::now();
	int num_classifier = pred->dim(1);
	float pixel_pred[num_classifier];
	std::copy((float*)pred->memoryHandle()->ptr(),(float*)pred->memoryHandle()->ptr()+num_classifier,pixel_pred);
	
    delay_msg.proc_time = delay_msg.end_proc-delay_msg.start_proc;
    delay_msg.cam_frame_delay = delay_msg.end_proc-delay_msg.cam_frame;
    delay_pub.publish(delay_msg);
#pragma unroll
    for(int i=0;i<5;i++){
#pragma unroll
        for(int j=0;j<2;j++){
            leftPixels[(i*2+j)*3]=(pixel_pred[i+j*20]+0.5)*640;
            leftPixels[1+(i*2+j)*3]=(pixel_pred[i+j*20+10]+0.5)*480;
            rightPixels[(i*2+j)*3]=(pixel_pred[i+j*20+5]+0.5)*640;
            rightPixels[1+(i*2+j)*3]=(pixel_pred[i+j*20+10+5]+0.5)*480;
        }
    }
#pragma unroll
    for(int i=0;i<10;i++){
        leftPos[i*3] = (leftPixels[i*3]*invKK[0]*2+invKK[2])*zPos[i];
        leftPos[i*3+1] = (leftPixels[i*3+1]*invKK[4]*2+invKK[5])*zPos[i];
        leftPos[i*3+2] = zPos[i];
        leftPosVec[i].setX(leftPos[i*3]);
        leftPosVec[i].setY(leftPos[i*3+1]);
        leftPosVec[i].setZ(zPos[i]);

        rightPos[i*3] = (rightPixels[i*3]*invKK[0]*2+invKK[2])*zPos[i];
        rightPos[i*3+1] = (rightPixels[i*3+1]*invKK[4]*2+invKK[5])*zPos[i];
        rightPos[i*3+2] = zPos[i];
        rightPosVec[i].setX(rightPos[i*3]);
        rightPosVec[i].setY(rightPos[i*3+1]);
        rightPosVec[i].setZ(zPos[i]);

    }

#pragma unroll
    for(int i=0;i<10;i++){
#pragma unroll
        leftPosVec[i] = camToVeh*leftPosVec[i];
        rightPosVec[i] = camToVeh*rightPosVec[i];

        leftPos[i*3] = leftPosVec[i].x();
        leftPos[i*3+1] = leftPosVec[i].y();
        leftPos[i*3+2] = leftPosVec[i].z();
        rightPos[i*3] = rightPosVec[i].x();
        rightPos[i*3+1] = rightPosVec[i].y();
        rightPos[i*3+2] = rightPosVec[i].z();
        for(int j=0;j<3;j++){/*
            float sumL = 0.0f;
            float sumR = 0.0f;
#pragma unroll
            for(int k=0;k<3;k++){
                sumL+=R[j+k*3]*leftPos[i*3+k];
                sumR+=R[j+k*3]*rightPos[i*3+k];
            }
            leftPos[i*3+j] = sumL;
            rightPos[i*3+j] = sumR;*/
            //std::cout<<rightPos[i*3+j]<<" , ";
        }
        //std::cout<<leftPosVec[i].x()<<","<<leftPosVec[i].y()<<","<<leftPosVec[i].z()<<std::endl;
        
        //std::cout<<leftPosVec[i].x()<<","<<leftPosVec[i].y()<<","<<leftPosVec[i].z()<<std::endl;
        //std::cout<<std::endl;
    }
    //std::cout<<std::endl;
#pragma unroll
    for(int i=0;i<10;i++){
        cv::circle(frame,cv::Point(leftPixels[i*3],leftPixels[i*3+1]),2,cv::Scalar(0,255-i*25,i*25),-1,8);
        cv::circle(frame,cv::Point(rightPixels[i*3],rightPixels[i*3+1]),2,cv::Scalar(255-i*25,i*25,0),-1,8);
    }    
   
    lane_msg.cam_frame_delay = ros::Time::now()-msg->header.stamp;
    lane_msg.header.stamp = ros::Time::now();
    lane_msg.left = leftPos;
    lane_msg.right = rightPos;
    laneOutput_pub.publish(lane_msg);
	// Update GUI Window
	cv::imshow(OPENCV_WINDOW, frame);
	cv::waitKey(3);
    if(vidRec.isOpened()){
        vidRec.write(frame);
    }
}


int main(int argc,char** argv){
	ros::init(argc,argv,"LanePredictorNode");
	ros::NodeHandle nh_;
	cv::namedWindow(OPENCV_WINDOW);
    fastCppInit(&argc,&argv);
    arr = hostArrayAllocRM(DataType::FLOAT,DDim(3,640,480,1),0);
	synchronizeStream(0);
    if(argc>3){
        time_t rawTime;
        struct tm * timeInfo;
        char buffer[80];
        std::time(&rawTime);
        timeInfo = std::localtime(&rawTime);
        std::strftime(buffer,80,"%F-%H-%M-%S",timeInfo);
        std::string time(buffer);
        std::string suffix = "_"+time+".avi";
        vidRec.open(argv[3]+suffix,CV_FOURCC('M','P','4','V'),20,cv::Size(640,480));
    }

    predictor = new LanePredictor_Q50(&argc,argv,0);
	ros::Subscriber sub = nh_.subscribe(argv[2],1,lanePredictorCb);
	delay_pub = nh_.advertise<lane_detection::Delay>("lane_detection/Delay",100);
    laneOutput_pub = nh_.advertise<lane_detection::LaneOutput>("lane_detection/LaneOutput",100);
	ros::spin();
	cv::destroyWindow(OPENCV_WINDOW);
    fastCppShutdown();
	return 0;
}
