#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "VideoReader.h"

using namespace cv;

int main( int argc, char** argv )
{

    VideoReader reader(argv[1], argv[2]);

    reader.setFrame(800);

    namedWindow("video", CV_WINDOW_AUTOSIZE);
    while (true) {
        VideoReader::FrameReadType data = reader.getNextFrame();
        if (!boost::get<0>(data))
            break;

        imshow("video", boost::get<1>(data));
        waitKey(1);
    }
    
    return 0;
}
