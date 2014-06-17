#include <boost/foreach.hpp>

#include <iostream>
#include "utils/cv_utils.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string/predicate.hpp>

#include <limits>


void filter_pixels(const std::vector<cv::Point2f>& pixels, const cv::Mat& img, std::vector<cv::Point2f>& filtered_pixels)
{
    std::vector<int> filtered_indices;
    filter_pixels(pixels, img, filtered_pixels, filtered_indices);
}


void filter_pixels(const std::vector<cv::Point2f>& pixels, const cv::Mat& img, std::vector<cv::Point2f>& filtered_pixels, std::vector<int>& filtered_indices)
{
    filtered_pixels.clear();
    filtered_indices.clear();
    for (int k = 0; k < pixels.size(); k++)
    {
        cv::Point2f px = pixels[k];
        if (px.x >= img.cols || px.y >= img.rows || px.x < 0 || px.y < 0)
            continue;
        filtered_pixels.push_back(px);
        filtered_indices.push_back(k);
    }
}


void get_pixel_colors(const std::vector<cv::Point2f>& pixels, const cv::Mat& img, std::vector<cv::Vec3b>& colors)
{
    colors.clear();
    BOOST_FOREACH(cv::Point2f px, pixels)
    {
        int col = (int) px.x;
        int row = (int) px.y;
        cv::Vec3b bgr = img.at<cv::Vec3b>(row, col);
        colors.push_back(bgr);
    }
}


void set_pixel_colors(const std::vector<cv::Point2f>& pixels, const cv::Vec3b& color, cv::Mat& img, int width)
{
    for (int k = 0; k < pixels.size(); k++)
    {
        int col = (int) pixels[k].x;
        int row = (int) pixels[k].y;
        img.at<cv::Vec3b>(row, col) = color;

        for (int dr = -width / 2; dr < width / 2; dr++)
        {
            for (int dc = -width / 2; dc < width / 2; dc++)
            {
                if (row + dr >= img.rows || row + dr < 0 || col + dc > img.cols || col + dc < 0)
                    continue;
                img.at<cv::Vec3b>(row + dr, col + dc) = color;
            }
        }
    }
}


bbox compute_bbox(const std::vector<cv::Point2f>& pixels)
{
    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::min();
    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::min();
    for (int k = 0; k < pixels.size(); k++)
    {
        x_min = std::min(x_min, pixels[k].x);
        x_max = std::max(x_max, pixels[k].x);
        y_min = std::min(y_min, pixels[k].y);
        y_max = std::max(y_max, pixels[k].y);
    }

    bbox box(x_min, x_max, y_min, y_max);
    return box;
}


void draw_bbox(const bbox& box, const cv::Scalar& color, cv::Mat& img, int line_width)
{
    // Draw rectangle
    if (!(box.x_min == img.cols))  // FIXME Temporary hack
        cv::rectangle(img, cv::Point(box.x_min, box.y_min),
                cv::Point(box.x_max, box.y_max), color, line_width, 8, 0);
}


void draw_bbox_3d_from_corner_pixels(const std::vector<cv::Point2f>& corner_pixels, const cv::Scalar& color, cv::Mat& img, int line_width)
{
    std::vector<cv::Point2f> corner_pixels_top(&corner_pixels[0], &corner_pixels[4]);
    std::vector<cv::Point2f> corner_pixels_bottom(&corner_pixels[4], &corner_pixels[8]);
    for (int k = 0; k < 4; k++)
        cv::line(img, corner_pixels[k], corner_pixels[k+4], color, line_width, 8, 0);
    for (int k = 0; k < 4; k++)
        cv::line(img, corner_pixels[k*2], corner_pixels[k*2+1], color, line_width, 8, 0);
    cv::line(img, corner_pixels[0], corner_pixels[3], color, line_width, 8, 0);
    cv::line(img, corner_pixels[1], corner_pixels[2], color, line_width, 8, 0);
    cv::line(img, corner_pixels[4], corner_pixels[7], color, line_width, 8, 0);
    cv::line(img, corner_pixels[5], corner_pixels[6], color, line_width, 8, 0);
}
