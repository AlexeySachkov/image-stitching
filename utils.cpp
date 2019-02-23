#include "utils.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cmath>

const cv::Point2f &getPoint(const std::vector<cv::Point2f> &points,
    size_t stride, size_t y, size_t x) {
  return points[y * stride + x];
}

const cv::Size calculateSizeForDisplaying(const cv::Size &originalSize,
    const cv::Size &screenSize, float C) {
  float hRatio = (float)originalSize.height / (float)screenSize.height;
  float wRatio = (float)originalSize.width / (float)screenSize.width;

  float ratio = fmax(hRatio, wRatio) + C;
  return cv::Size(round((float)originalSize.height / ratio),
      round((float)originalSize.width / ratio));
}

void displayResult(const std::string &windowName, const cv::Mat &result) {
  cv::Mat resized;
  cv::resize(result, resized, calculateSizeForDisplaying(result.size()));
  cv::imshow(windowName, result);
}
