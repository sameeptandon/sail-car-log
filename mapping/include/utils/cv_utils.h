#pragma once

#include <opencv2/core/core.hpp>

// Get rid of pixels outside of image
void filter_pixels(const std::vector<cv::Point2f>& pixels, const cv::Mat& img, std::vector<cv::Point2f>& filtered_pixels);
void filter_pixels(const std::vector<cv::Point2f>& pixels, const cv::Mat& img, std::vector<cv::Point2f>& filtered_pixels, std::vector<int>& filtered_indices);

// Call filter_pixels before using
void get_pixel_colors(const std::vector<cv::Point2f>& pixels, const cv::Mat& img, std::vector<cv::Vec3b>& colors);

// Useful for debugging projections
void set_pixel_colors(const std::vector<cv::Point2f>& pixels, const cv::Vec3b& color, cv::Mat& img, int width=1);

void draw_bbox_from_pixels(const std::vector<cv::Point2f>& pixels, const cv::Scalar& color, cv::Mat& img, int line_width);

void draw_bbox_3d_from_corner_pixels(const std::vector<cv::Point2f>& corner_pixels, const cv::Scalar& color, cv::Mat& img, int line_width);
