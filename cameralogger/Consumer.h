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

using namespace FlyCapture2; 
using namespace std; 
// Consumer thread class
template <typename T>
class Consumer
{
    private:
        ///////////////////////////////////////////////////////////////////////////////////////
        void writeToDisk (const T* obj) {
                FPS_CALC (_aviFileName, _buf);
                Error error;
                error = _aviRecorder.AVIAppend((T*)obj);
                if (error != PGRERROR_OK) 
                    PrintError(error);
                delete obj;
        }

        ///////////////////////////////////////////////////////////////////////////////////////
        // Consumer thread function
        void receiveAndProcess () {
                while (!_is_done) {
                    if (_buf->isEmpty()) {
                        usleep(1000 / 120); // poll at max of 120hz
                    } else { 
                        writeToDisk (_buf->getFront ());
                    }
                }

                boost::mutex::scoped_lock io_lock (*_io_mutex);
                printf("Writing remaning %d images in the buffer to disk...\n", _buf->getSize ());
                while (!_buf->isEmpty ()) { 
                    writeToDisk (_buf->getFront ());
                }
        }

    public:
        Consumer (SynchronizedBuffer<T> *buf, std::string aviFileName, boost::mutex* io_mutex,
                float frameRate, int imWidth, int imHeight)
            : _buf (buf), _aviFileName(aviFileName), _io_mutex(io_mutex),  _frameRate(frameRate)
        {

            Error error;
            _is_done = false; 

            // Uncompressed Options
            AVIOption option;

            //MJPG Compressed Options
            //MJPGOption option;
            //option.quality = 50;

            //H264 Compressed Options
            //H264Option option;
            //option.bitrate = 1000000;
            //option.height = imHeight;
            //option.width = imWidth;

            cout << "frameRate = " << _frameRate << endl; 
            option.frameRate = _frameRate;
            error = _aviRecorder.AVIOpen(_aviFileName.c_str(), &option); 
            if (error != PGRERROR_OK) { 
                PrintError(error);
            }

            _thread.reset (new boost::thread (boost::bind (&Consumer::receiveAndProcess, this)));
        }

        ///////////////////////////////////////////////////////////////////////////////////////
        void stop ()  {
            printf("stop called\n");
            _is_done = true; 
            _thread->join ();
            boost::mutex::scoped_lock io_lock (*_io_mutex);
            _aviRecorder.AVIClose();
            printf("Consumer done.\n");
        }

    private:
        SynchronizedBuffer<T>* _buf;
        boost::shared_ptr<boost::thread> _thread;
        string _aviFileName; 
        boost::mutex* _io_mutex; 
        float _frameRate;
        AVIRecorder _aviRecorder;
        bool _is_done; 
};

