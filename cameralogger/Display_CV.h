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

using namespace FlyCapture2; 
using namespace std;
using namespace cv;


// Display thread class
template <typename T>
class Display 
{
    private:
        ///////////////////////////////////////////////////////////////////////////////////////
        void show (const T* obj) {
            FPS_CALC (_name, _buf);
            _img = cvCreateImage(cvSize(obj->GetCols(), obj->GetRows()), IPL_DEPTH_8U, 3);
            _img->height = obj->GetRows();
            _img->width = obj->GetCols();
            _img->widthStep = obj->GetStride();
            _img->nChannels = 3;
            _img->imageData = (char*)obj->GetData();
            imshow(_name.c_str(), Mat(_img));

            //cvShowImage(_name.c_str(), _img);
            cvWaitKey(10);
            cvReleaseImage(&_img);
            delete obj;
        }

        ///////////////////////////////////////////////////////////////////////////////////////
        // Display thread function
        void receiveAndProcess () {
                while (!_is_done) {
                    if (_buf->isEmpty()) {
                        //usleep(1000 / 120); // poll at max of 120hz
                    } else { 
                        while (!_buf->isEmpty ()) 
                            show(_buf->getFront ());
                    }
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
        Display(SynchronizedBuffer<T> *buf, std::string name, boost::mutex* io_mutex)
            : _buf (buf), _name(name), _io_mutex(io_mutex) 
        {

            _is_done = false; 

            cvNamedWindow(_name.c_str(), CV_WINDOW_AUTOSIZE);

            _thread.reset (new boost::thread (boost::bind (&Display::receiveAndProcess, this)));
        }

        ///////////////////////////////////////////////////////////////////////////////////////
        void stop ()  {
            printf("stop called\n");
            _is_done = true; 
            //_thread->join ();
            boost::mutex::scoped_lock io_lock (*_io_mutex);
            printf("Display done.\n");
        }

    private:
        SynchronizedBuffer<T>* _buf;
        boost::shared_ptr<boost::thread> _thread;
        string _name; 
        boost::mutex* _io_mutex; 
        float _frameRate;
        VideoWriter _writer;
        bool _is_done;
        IplImage* _img;
        Image* _tmp;
};

