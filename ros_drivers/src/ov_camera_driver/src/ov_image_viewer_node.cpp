#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>

static const std::string OPENCV_WINDOW = "Image window";

using namespace std;

class ImageConverter
{
    ros::Subscriber image_sub_;
    string data_dir;
    string cam_prefix;
    uint64_t frame_count;


    public:
    ImageConverter(ros::NodeHandle nh)
    {
        nh.param<string>("outdir", data_dir, string("data"));
        nh.param<string>("prefix", cam_prefix, string("cam"));
        frame_count = 0;

        string topic;
        nh.param<string>("image", topic, string(""));
        ROS_INFO_STREAM("Subscribing to topic " << topic << "...");
        // Subscrive to input video feed and publish output video feed
        image_sub_ = nh.subscribe(topic, 1000, &ImageConverter::imageCb, this);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::CompressedImage& msg)
    {
        frame_count++;
        cout << "received " << msg.data.size() << endl;
        ofstream imgFile;
        string fname = data_dir + "/" + boost::lexical_cast<string>(frame_count) + "_" + cam_prefix + ".jpg";
        imgFile.open(fname, ios::out | ios::binary);
        imgFile.write((char*)msg.data.data(),msg.data.size());
        imgFile.close();

        cv::Mat img = cv::imdecode(msg.data, CV_LOAD_IMAGE_COLOR);
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
    ImageConverter ic(nh);

    ros::spin();
    return 0;
}
