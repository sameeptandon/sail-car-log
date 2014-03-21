#pragma once

#include "FlyCapture2.h"
#include "CameraHelper.h"
#include "../util/synchronized_buffer.h"
#include "../util/fps_calc.h"
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <string>
#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace FlyCapture2; 
using namespace std;
using namespace cv;

// Consumer thread class
template <typename T>
class Consumer
{
    private:
        ///////////////////////////////////////////////////////////////////////////////////////
        void writeToDisk (const T* obj) {
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
        }

        ///////////////////////////////////////////////////////////////////////////////////////
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
        


        void toOpenCv(Image* fly_img, IplImage* cv_img) {
            Image _tmp; 
            fly_img->Convert(PIXEL_FORMAT_BGR, &_tmp);

            cv_img->height = _tmp.GetRows();
            cv_img->width = _tmp.GetCols();
            cv_img->widthStep = _tmp.GetStride();
            cv_img->nChannels = 3;
            cv_img->imageData = (char*)_tmp.GetData();
        }

    public:
        Consumer (SynchronizedBuffer<T> *buf, std::string aviFileName,
                boost::mutex* io_mutex, float frameRate, int imWidth, int
                imHeight)
            : _buf (buf), _aviFileName(aviFileName), _io_mutex(io_mutex),
            _frameRate(frameRate)
        {

            Error error;
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
                cerr << "File not opened!" << endl; 
            }

            _thread.reset (new boost::thread (boost::bind (&Consumer::receiveAndProcess, this)));
        }

        ///////////////////////////////////////////////////////////////////////////////////////
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
        IplImage* _img;
        Image* _tmp;
        uint64_t framesConsumed; 
};

