#pragma once

#include <opencv2/core/core.hpp>

// Get rid of pixels outside of image
void filter_pixels(const std::vector<cv::Point2f>& pixels, const cv::Mat& img, std::vector<cv::Point2f>& filtered_pixels);
void filter_pixels(const std::vector<cv::Point2f>& pixels, const cv::Mat& img, std::vector<cv::Point2f>& filtered_pixels, std::vector<int>& filtered_indices);

// Call filter_pixels before using
void get_pixel_colors(const std::vector<cv::Point2f>& pixels, const cv::Mat& img, std::vector<cv::Vec3b>& colors);

// Useful for debugging projections
void set_pixel_colors(const std::vector<cv::Point2f>& pixels, const cv::Vec3b& color, cv::Mat& img, int width=1);

struct bbox
{
    float x_min, x_max, y_min, y_max;

    bbox(float x1, float x2, float y1, float y2) : x_min(x1), x_max(x2),
                                                   y_min(y1), y_max(y2)
    { }

    float xy_ratio()
    {
        return (x_max - x_min) / (y_max - y_min);
    }
};

bbox compute_bbox(const std::vector<cv::Point2f>& pixels);

void draw_bbox(const bbox& box, const cv::Scalar& color, cv::Mat& img, int line_width);

void draw_bbox_3d_from_corner_pixels(const std::vector<cv::Point2f>& corner_pixels, const cv::Scalar& color, cv::Mat& img, int line_width);
