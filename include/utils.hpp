
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

std::pair<cv::Point2f, cv::Point2f> getTwoBottomLeftPoints(
    const std::vector<std::vector<cv::Point2f>> &points);

std::vector<cv::Point2f> extractCorners(
    const std::vector<std::vector<cv::Point2f>> &points);

std::vector<cv::Point2f> extractCorners(const cv::Mat &image);

struct corners_info_t {
  corners_info_t() = delete;
  corners_info_t(const std::vector<cv::Point2f> &corners) {
    minX = fmin(fmin(corners[0].x, corners[1].x),
        fmin(corners[2].x, corners[3].x));
    maxX = fmax(fmax(corners[0].x, corners[1].x),
        fmax(corners[2].x, corners[3].x));
    minY = fmin(fmin(corners[0].y, corners[1].y),
        fmin(corners[2].y, corners[3].y));
    maxY = fmax(fmax(corners[0].y, corners[1].y),
        fmax(corners[2].y, corners[3].y));

    width = maxX - minX;
    height = maxY - minY;
  }

  float minX, maxX, minY, maxY;
  float width, height;
};

#endif // __INCLUDE_UTILS_HPP__
