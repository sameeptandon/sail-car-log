#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "VideoReader.h"

using namespace cv;

int main( int argc, char** argv )
{

    VideoReader reader(argv[1], argv[2]);

    reader.setFrame(800);

    namedWindow("video", CV_WINDOW_AUTOSIZE);
    cv::Mat frame;
    while (true) {
        bool success = reader.getNextFrame(frame);
        if (!success)
            break;
        imshow("video", frame);
        waitKey(1);
    }

    return 0;
}
