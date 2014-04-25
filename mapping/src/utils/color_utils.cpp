#include <opencv2/opencv.hpp>
#include "utils/color_utils.h"

void heatmap_color(std::vector<float>& intensities, cv::Mat& rgb, int minc, int maxc)
{
    float s = (1.0 / (maxc - minc));

    cv::Mat hsv(intensities.size(), 1, CV_8UC3);
    for (int k = 0; k < intensities.size(); k++)
    {
        unsigned char h = (unsigned char) (s * intensities[k] * 255 * 0.4);
        hsv.at<cv::Vec3b>(k, 0) = cv::Vec3b(h, 255, 255);
    }

    cv::cvtColor(hsv, rgb, CV_HSV2RGB);
}

