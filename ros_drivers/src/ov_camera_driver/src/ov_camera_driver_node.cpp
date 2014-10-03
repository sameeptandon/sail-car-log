#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_publisher.h>
#include <opencv2/core/core.hpp>
#include <libuvc/libuvc.h>
#include <string>
#include <stdexcept>

// this driver will work only for OV cameras running MJPEG at 1280x800@10 fps

#define OV_VENDOR_ID 0x05a9
#define IMAGE_FRAME_WIDTH 1280
#define IMAGE_FRAME_HEIGHT 800
#define IMAGE_FRAME_RATE 10

using namespace std;

struct CameraController {
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;
    uvc_stream_handle_t *strmh;
};

unsigned int hexStringToUnsignedInt(string input) {
    std::stringstream ss;
    ss << std::hex << input;
    unsigned int out;
    ss >> out;
    return out; 
}

void checkError(uvc_error_t& res, string message) { 
    if (res < 0) {
        uvc_perror(res, message.c_str());
        std::runtime_error(message.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ov_camera_driver");
    ros::NodeHandle nh("~");

    //setup the ros publisher
    //image_transport::ImageTransport it(nh);
    ros::Publisher pub;
    pub = nh.advertise<sensor_msgs::CompressedImage>("/" +
            ros::this_node::getNamespace() + "/image_raw", 1000);

    // grab the product id
    string product_id_input; 
    nh.param<string>("pid", product_id_input, "NONE");
    cout << product_id_input << endl;
    int product_id = hexStringToUnsignedInt(product_id_input);
    
    uvc_context_t *ctx;
    uvc_error_t res;
    CameraController camera;
   
    // initialize uvc context
    res = uvc_init(&ctx, NULL);
    checkError(res, "uvc_init");
    cout << "UVC initialized" << endl;

    // connect to camera
    res = uvc_find_device(ctx, &camera.dev,
            OV_VENDOR_ID, product_id, NULL);
    checkError(res, "uvc_find_device");
    cout << "Device Found" << endl; 

    //configure and start the stream
    res = uvc_open(camera.dev, &camera.devh);
    checkError(res, "uvc_open"); 
    cout << "Device Opened" << endl;

    uvc_print_diag(camera.devh, stderr);

    res = uvc_get_stream_ctrl_format_size(
            camera.devh, &camera.ctrl, 
            UVC_FRAME_FORMAT_MJPEG, IMAGE_FRAME_WIDTH,
            IMAGE_FRAME_HEIGHT, IMAGE_FRAME_RATE 
            );
    checkError(res, "get_mode");
    cout << "Mode set" << endl; 
    uvc_print_stream_ctrl(&camera.ctrl, stderr);

    res = uvc_stream_open_ctrl(camera.devh, &camera.strmh,
            &camera.ctrl);
    checkError(res, "open_ctrl");
    cout << "Stream ctrl open" << endl; 

    // start the stream
    res = uvc_stream_start_iso(camera.strmh, NULL, NULL);
    checkError(res, "start_iso"); 
    cout << "Stream open" << endl; 


    // poll the cameras
    uvc_frame_t *frame;
    while (ros::ok()) {
        //grab a frame
        res = uvc_stream_get_frame(camera.strmh, &frame, 0);
        checkError(res, "get_frame");
        if (frame == NULL) { 
            ROS_INFO_STREAM(ros::this_node::getNamespace() << " frame is null; skipping");
            continue;
        }

        //ROS_INFO_STREAM(ros::this_node::getNamespace() << " captured: " << frame->data_bytes);
        
        //good frame received
        sensor_msgs::CompressedImage msg;
        msg.header.stamp = ros::Time::now();
        msg.format = "jpeg";
        msg.data.resize(frame->data_bytes);
        memcpy(&msg.data[0], frame->data, frame->data_bytes);
        pub.publish(msg);

    }

    //cleanup
    uvc_stream_close(camera.strmh); 
    uvc_close(camera.devh);
    uvc_unref_device(camera.dev);
    uvc_exit(ctx);
    cout << "ov camera driver exited cleanly" <<
        endl;
}
