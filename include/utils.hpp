
#ifndef __INCLUDE_UTILS_HPP__
#define __INCLUDE_UTILS_HPP__

#include "opencv2/core/core.hpp"

#include <vector>
#include <string>

void displayResult(
    const std::string &windowName, const cv::Mat &result, bool wait = false);

std::vector<cv::Point2f> extractCorners(
    const std::vector<std::vector<cv::Point2f>> &points);

std::vector<cv::Point2f> extractCorners(const cv::Mat &image);

std::vector<cv::Point2f> extractCorners(const cv::Size &size);

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

    width = maxX - (minX < 0 ? minX : 0);
    height = maxY - (minY < 0 ? minY : 0);
  }

  float minX, maxX, minY, maxY;
  float width, height;
};

bool projectToTheFloor(const cv::Mat &image, const cv::Size &chessboardSize,
    cv::Mat &result, std::vector<cv::Point2f> &chessboardCornersOrig,
    std::vector<cv::Point2f> &chessboardCorners,
    std::vector<cv::Point2f> &imageCorners);

void computeHomography(const std::vector<cv::Point2f> &from,
    const std::vector<cv::Point2f> &to, const cv::Size &size_from,
    cv::Mat &H, cv::Size &shift, std::vector<cv::Point2f> &imageCorners);

#endif // __INCLUDE_UTILS_HPP__
