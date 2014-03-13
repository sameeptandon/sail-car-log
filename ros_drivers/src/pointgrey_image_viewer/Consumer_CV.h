#pragma once

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
            cout << obj->size().width << endl; 
            cout << obj->size().height << endl; 
            cout << framesConsumed << endl; 
            Mat imgMat(*obj);
            _writer << imgMat; 
            delete obj; 
            framesConsumed++; 
            /*
            //FPS_CALC (_aviFileName, _buf);
            _img = cvCreateImage(cvSize(obj->GetCols(), obj->GetRows()), IPL_DEPTH_8U, 3);
            _img->height = obj->GetRows();
            _img->width = obj->GetCols();
            _img->widthStep = obj->GetStride();
            _img->nChannels = 3;
            _img->imageData = (char*)obj->GetData();
            Mat imgMat(_img);
            _writer << imgMat;
            delete obj;
            framesConsumed++;
            */
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
                //printf("Writing remaning %d images in the buffer to disk...\n", _buf->getSize ());
                while (!_buf->isEmpty ()) { 
                    writeToDisk (_buf->getFront ());
                }
        }
        

    public:
        Consumer (SynchronizedBuffer<T> *buf, std::string aviFileName,
                boost::mutex* io_mutex, float frameRate, int imWidth, int
                imHeight)
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

            _thread.reset (new boost::thread (boost::bind (&Consumer::receiveAndProcess, this)));
        }

        void stop ()  {
            //printf("stop called\n");
            _is_done = true; 
            _thread->join ();
            boost::mutex::scoped_lock io_lock (*_io_mutex);
            //printf("Consumer done.\n");
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
};

