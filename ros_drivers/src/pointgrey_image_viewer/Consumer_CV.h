#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "synchronized_buffer.h"
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <string>
#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

// Consumer thread class
template <typename T>
class Consumer
{
    private:
        void writeToDisk (const T* obj) {
            _writer << *obj;
            delete obj; 
            framesConsumed++;
            std_msgs::String msg;
            msg.data = "done with frame";
            _writer_ack_pub.publish(msg);
        }

        // Consumer thread function
        void receiveAndProcess () {
            while (!_is_done) {
                if (_buf->isEmpty()) {
                    usleep(1000 / 200); // poll at max of 120hz
                } else { 
                    while (!_buf->isEmpty ())  {
                        int buf_size = _buf->getSize();
                        for (int j = 0; j < buf_size; j++)
                            writeToDisk (_buf->getFront ());
                    }

                }
            }

            boost::mutex::scoped_lock io_lock (*_io_mutex);
            while (!_buf->isEmpty ()) { 
                writeToDisk (_buf->getFront ());
            }
        }


    public:
        Consumer (SynchronizedBuffer<T> *buf, std::string aviFileName,
                boost::mutex* io_mutex, float frameRate, int imWidth, int
                imHeight, ros::NodeHandle nh)
            : _buf (buf), _aviFileName(aviFileName), _io_mutex(io_mutex),
            _frameRate(frameRate)
    {

        _is_done = false;
        framesConsumed = 0; 

        _writer = VideoWriter(_aviFileName.c_str(),
                //CV_FOURCC('X','V','I','D'),
                CV_FOURCC('F','M','P','4'),
                //CV_FOURCC('U','2','6','3'),
                //CV_FOURCC('M','P','E','G'),
                _frameRate,
                cvSize(imWidth, imHeight));

        if (!_writer.isOpened()) {
            cout << "File not opened!" << endl; 
        }

        _writer_ack_pub = nh.advertise<std_msgs::String>("writer_ack",1000); 

        _thread.reset (new boost::thread (boost::bind (&Consumer::receiveAndProcess, this)));
    }

        void stop ()  {
            _is_done = true; 
            _thread->join ();
            boost::mutex::scoped_lock io_lock (*_io_mutex);
        }

        uint64_t getNumFramesConsumed() { 
            return framesConsumed;
        }

    private:
        SynchronizedBuffer<T>* _buf;
        boost::shared_ptr<boost::thread> _thread;
        string _aviFileName; 
        boost::mutex* _io_mutex; 
        float _frameRate;
        VideoWriter _writer;
        bool _is_done;
        uint64_t framesConsumed;
        ros::Publisher _writer_ack_pub; 
};

