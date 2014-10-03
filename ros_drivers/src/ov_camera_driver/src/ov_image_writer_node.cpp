#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_publisher.h>
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

using namespace std;

class ImageConverter
{
    ros::Subscriber image_sub_;
    string basename;
    uint64_t frame_count;
    int camera_num;
    ofstream img_ros_acq_time_file; 
    string img_dir; 

    public:
    ImageConverter(ros::NodeHandle& nh)
    {
        nh.param<string>("basename", basename, string(""));
        nh.param<int>("cameranum", camera_num, 0);
        frame_count = 0;
        stringstream rostime_filename;
        rostime_filename << basename << camera_num << "_rostime.txt";
        img_ros_acq_time_file.open(rostime_filename.str().c_str());
        stringstream img_dirname;
        img_dirname << basename << camera_num;
        img_dir = img_dirname.str();
        boost::filesystem::path dir(img_dir.c_str());
        if (boost::filesystem::create_directory(dir)) { 
            cout << "Image folder create success" << endl; 
        }

        // Subscrive to input video feed 
        string topic;
        nh.param<string>("image", topic, string(""));
        ROS_INFO_STREAM("Subscribing to topic " << topic << "...");
        image_sub_ = nh.subscribe(topic, 10000, &ImageConverter::imageCb, this);

    }

    void imageCb(const sensor_msgs::CompressedImage& msg)
    {
        frame_count++;
        cout << "received " << msg.data.size() << endl;
        ofstream imgFile;
        string fname = img_dir + "/" + boost::lexical_cast<string>(frame_count) + ".jpg";
        imgFile.open(fname, ios::out | ios::binary);
        imgFile.write((char*)msg.data.data(),msg.data.size());
        imgFile.close();
        img_ros_acq_time_file << msg.header.stamp << std::endl;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_writer");
    ros::NodeHandle nh("~");
    ImageConverter ic(nh);
    ros::spin();
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(2.0));
    return 0;
}
