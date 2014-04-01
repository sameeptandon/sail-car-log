#include "FlyCapture2.h"
#include <string>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "util/synchronized_buffer.h"
#include "util/time.h"
#include "util/fps_calc.h"
#include "Consumer_CV.h"
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "zmq.hpp"
#include "GPSRecord.h"

using namespace FlyCapture2;
using namespace std; 

typedef boost::posix_time::ptime Time;
typedef boost::posix_time::time_duration TimeDuration; 

class SyncBuffer {
    private:
        boost::mutex _io_mutex; 
        SynchronizedBuffer<Image>* _buffer;
    public:
        SyncBuffer () {
            _buffer = new SynchronizedBuffer<Image>(&_io_mutex);
        }
        boost::mutex* getMutex() { return & _io_mutex; }
        SynchronizedBuffer<Image>* getBuffer() { return _buffer; } 
};

void convertToCV(const Image* obj, IplImage* img) {
    img->height = obj->GetRows();
    img->width = obj->GetCols();
    img->widthStep = obj->GetStride();
    img->nChannels = 3;
    img->imageData = (char*)obj->GetData();
}

void show (IplImage* img, string name) {
    Mat imout;
    resize(Mat(img), imout, Size(320,240));
    imshow(name.c_str(), imout);
}
