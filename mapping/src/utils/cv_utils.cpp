#include <boost/foreach.hpp>

#include <iostream>
#include "utils/cv_utils.h"

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


void draw_bbox_from_pixels(const std::vector<cv::Point2f>& pixels, const cv::Scalar& color, cv::Mat& img, int line_width)
{
    // Compute bbox
    float x_min = img.cols;
    float x_max = 0;
    float y_min = img.rows;
    float y_max = 0;
    for (int k = 0; k < pixels.size(); k++)
    {
        x_min = std::min(x_min, pixels[k].x);
        x_max = std::max(x_max, pixels[k].x);
        y_min = std::min(y_min, pixels[k].y);
        y_max = std::max(y_max, pixels[k].y);
    }
    // Draw rectangle
    cv::rectangle(img, cv::Point(x_min, y_min), cv::Point(x_max, y_max), color, line_width, 8, 0);
}
