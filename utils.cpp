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

static void getSteps(bool byRow, const cv::Size &bs, const cv::Point2f &first,
    const cv::Point2f &second, const cv::Point2f &third, int &istart,
    int &iend, int &istep, int &jstart, int &jend, int &jstep) {
  if (byRow) {
    // A B
    // C
    // where A - first, B - second, C - third
    // AB - row
    // AC - column
    int w = (byRow) ? bs.width : bs.height;
    int h = (byRow) ? bs.height : bs.width;
    if (first.x < second.x) {
      // columns goes from left to right
      jstart = 0;
      jend = w;
      jstep = 1;
    }
    else {
      // columns goes from right to left
      jstart = w - 1;
      jend = -1;
      jstep = -1;
    }

    if (first.y < third.y) {
      // rows goes from top to bottom
      istart = 0;
      iend = h;
      istep = 1;
    }
    else {
      // rows goes from bottom to top
      istart = h - 1;
      iend = -1;
      istep = -1;
    }
  } else {
    // by column
    // A B
    // C
    // where A - first, B - second, C - third
    // AB - column
    // AC - row
    int w = (byRow) ? bs.width : bs.height;
    int h = (byRow) ? bs.height : bs.width;
    if (first.x > third.x) {
      // columns goes from left to right
      jstart = 0;
      jend = w;
      jstep = 1;
    }
    else {
      // columns goes from right to left
      jstart = w - 1;
      jend = -1;
      jstep = -1;
    }

    if (first.y < second.y) {
      // rows goes from top to bottom
      istart = 0;
      iend = h;
      istep = 1;
    }
    else {
      // rows goes from bottom to top
      istart = h - 1;
      iend = -1;
      istep = -1;
    }
  }
}

static void getSteps(const std::vector<cv::Point2f> &p, const cv::Size &bs,
    bool &byRow, int &istart, int &iend, int &istep, int &jstart, int &jend,
    int &jstep) {
  byRow = true;
  for (int i = 1; i < bs.width; ++i) {
    if (p[i - 1].x > p[i].x) {
      byRow = false;
    }
  }
  if (!byRow) {
    byRow = true;
    for (int i = 1; i < bs.width; ++i) {
      if (p[i - 1].x < p[i].x) {
        byRow = false;
      }
    }
  }

  if (byRow) {
    cv::Point2f first = getPoint(p, bs.width, 0, 0);
    cv::Point2f second = getPoint(p, bs.width, 0, 1);
    cv::Point2f third = getPoint(p, bs.width, 1, 0);
    getSteps(true, bs, first, second, third, istart, iend, istep, jstart,
        jend, jstep);
  } else {
    // by column
    cv::Point2f first = getPoint(p, bs.height, 0, 0);
    cv::Point2f second = getPoint(p, bs.height, 0, 1);
    cv::Point2f third = getPoint(p, bs.height, 1, 0);
    getSteps(false, bs, first, second, third, istart, iend, istep, jstart,
        jend, jstep);
  }
}

std::vector<std::vector<cv::Point2f>> orderChessboardCorners(
    const std::vector<cv::Point2f> &chessboardCorners,
    const cv::Size &boardSize) {
  std::vector<std::vector<cv::Point2f>> result(boardSize.height,
      std::vector<cv::Point2f>(boardSize.width));

  bool isByRow = false;
  int jstart, jend, jstep;
  int istart, iend, istep;
  getSteps(chessboardCorners, boardSize, isByRow, istart, iend, istep, jstart,
      jend, jstep);

  if (isByRow) {
    for (int i = 0, ii = istart; ii != iend; ++i, ii += istep) {
      for (int j = 0, jj = jstart; jj != jend; ++j, jj += jstep) {
        result[i][j] = getPoint(chessboardCorners, boardSize.width, ii, jj);
      }
    }
  } else {
    for (int j = 0, jj = jstart; jj != jend; ++j, jj += jstep) {
      for (int i = 0, ii = istart; ii != iend; ++i, ii += istep) {
        result[i][j] = getPoint(chessboardCorners, boardSize.height, ii, jj);
      }
    }
  }

  return result;
}
