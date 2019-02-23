
#ifndef __INCLUDE_UTILS_HPP__
#define __INCLUDE_UTILS_HPP__

#include "opencv2/core/core.hpp"

#include <vector>
#include <string>

const cv::Point2f &getPoint(const std::vector<cv::Point2f> &points,
    size_t stride, size_t y, size_t x);

const cv::Size calculateSizeForDisplaying(const cv::Size &originalSize,
    const cv::Size &screenSize = cv::Size(1920, 1080), float C = 2.2);

void displayResult(const std::string &windowName, const cv::Mat &result);

std::vector<std::vector<cv::Point2f>> orderChessboardCorners(
    const std::vector<cv::Point2f> &chessboardCorners,
    const cv::Size &boardSize);

#endif // __INCLUDE_UTILS_HPP__
