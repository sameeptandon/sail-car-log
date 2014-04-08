#pragma once
#include "opencv2/opencv.hpp"
#include <string.h>
#include <boost/tuple/tuple.hpp>

#define NUM_SPLITS 10

using namespace cv;
using namespace std;

class VideoReader{ 
    public:
        typedef boost::tuple<bool,Mat> FrameReadType; 
        VideoReader(string path, string base_video_name);
        FrameReadType getNextFrame();
        void setFrame(int fnum);
    
    protected:
        int framenum;
        vector<VideoCapture> captures;

};
