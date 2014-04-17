#pragma once
#include <string>
#include <boost/tuple/tuple.hpp>
#include <opencv2/opencv.hpp>

#define NUM_SPLITS 10

using namespace cv;
using namespace std;

class VideoReader{
    public:
        typedef boost::tuple<bool,Mat> FrameReadType;
        VideoReader(string path, string base_video_name);
        bool skip(int k);
        bool getNextFrame(cv::Mat& frame);
        void setFrame(int fnum);
        int getFps();
        int getFrameWidth();
        int getFrameHeight();
        int getCodecType();

    protected:
        int framenum;
        vector<VideoCapture> captures;
};
