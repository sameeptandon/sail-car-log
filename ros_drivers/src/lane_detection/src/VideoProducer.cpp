#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class VideoProducer{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
	cv::VideoCapture* cap;

public:
  VideoProducer(std::string fname) : it_(nh_){
		cap = new cv::VideoCapture(fname);
		if(!cap->isOpened())
			ROS_ERROR("Failed to open video");
		image_pub_ = it_.advertise("/VideoProducer/output_video",1);
		//cv::namedWindow(OPENCV_WINDOW);
	}
	~VideoProducer(){
		//cv::destroyWindow(OPENCV_WINDOW);
	}

	bool readVideo(){
		cv::Mat frame;
		bool success = cap->read(frame);
		if(!success){
			ROS_ERROR("Failed to read frame");
            return false;
        }
		cv_bridge::CvImage cvi;
		cvi.encoding = "bgr8";
		cvi.image = frame;
        cvi.header.stamp = ros::Time::now();
		sensor_msgs::Image im;
		cvi.toImageMsg(im);
		image_pub_.publish(im);
		//cv_bridge::CvImagePtr cv_ptr;
		//cv_ptr = cv_bridge::toCvCopy(im, sensor_msgs::image_encodings::BGR8);
		//cv::imshow(OPENCV_WINDOW,cv_ptr->image);
		//cv::waitKey(3);
        return true;
	}
};

int main(int argc, char** argv){
	ros::init(argc,argv,"VideoProducer");
	VideoProducer producer(argv[1]);
	ros::Rate loop_rate(60);
	while(ros::ok()){
		if(!producer.readVideo())
            break;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
