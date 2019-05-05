#include "utils.hpp"
#include "opts.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cmath>

extern command_line_opts opts;

namespace {

const cv::Point2f &getPoint(const std::vector<cv::Point2f> &points,
    size_t stride, size_t y, size_t x) {
  return points[y * stride + x];
}

const cv::Size calculateSizeForDisplaying(const cv::Size &originalSize,
    const cv::Size &screenSize = cv::Size(1080, 1920)) {
  // To make looking at several images eaiser, each of
  // them should not occupy more than half of the screen.
  // For height this restriction is relaxed
  float targetH = (float)screenSize.height * 0.7;
  float targetW = (float)screenSize.width * 0.7;

  float hRatio = originalSize.height / targetH;
  float wRatio = originalSize.width / targetW;

  float ratio = fmax(hRatio, wRatio);
  return cv::Size((int)round(originalSize.height / ratio),
      (int)round(originalSize.width / ratio));
}

void getSteps(bool byRow, const cv::Size &bs, const cv::Point2f &first,
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

void getSteps(const std::vector<cv::Point2f> &p, const cv::Size &bs,
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

std::pair<cv::Point2f, cv::Point2f> getTwoBottomLeftPoints(
    const std::vector<std::vector<cv::Point2f>> &points) {
  return std::make_pair(points.back()[0], points.back()[1]);
}

}

void displayResult(
    const std::string &windowName, const cv::Mat &result, bool wait) {
  if (opts.interactive) {
    cv::Mat resized;
    cv::resize(result, resized, calculateSizeForDisplaying(result.size()));
    cv::imshow(windowName, resized);
    if (wait) {
      cv::waitKey();
    }
  }
}


std::vector<cv::Point2f> extractCorners(
    const std::vector<std::vector<cv::Point2f>> &points) {
  std::vector<cv::Point2f> result;
  result.push_back(points.back().front()); // bottom left
  result.push_back(points.back().back()); // bottom right
  result.push_back(points.front().back()); // top right
  result.push_back(points.front().front()); // top left

  return result;
}

std::vector<cv::Point2f> extractCorners(const cv::Mat &image) {
  return extractCorners(cv::Size(image.cols, image.rows));
}

std::vector<cv::Point2f> extractCorners(const cv::Size &size) {
  std::vector<cv::Point2f> result;
  result.push_back(cv::Point2f(0, size.height)); // bottom left
  result.push_back(cv::Point2f(size.width, size.height)); // bottom right
  result.push_back(cv::Point2f(size.width, 0)); // top right
  result.push_back(cv::Point2f(0, 0)); // top left
  return result;
}

bool projectToTheFloor(const cv::Mat &image, const cv::Size &chessboardSize,
    cv::Mat &result, std::vector<cv::Point2f> &chessboardCornersOrig,
    std::vector<cv::Point2f> &chessboardCorners,
    std::vector<cv::Point2f> &imageCorners) {
  std::vector<cv::Point2f> chessboardCornersTemp;
  // search for chessboard corners
  if (!cv::findChessboardCorners(image, chessboardSize, chessboardCornersTemp,
      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |
      CV_CALIB_CB_NORMALIZE_IMAGE))
    return false;

  // optimize results
  cv::Mat viewGray;
  cv::cvtColor(image, viewGray, CV_BGR2GRAY);
  cv::cornerSubPix(viewGray, chessboardCornersTemp, cv::Size(11, 11),
      cv::Size(-1, -1),
      cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

  cv::Mat temp;
  image.copyTo(temp);
#if 0
  cv::drawChessboardCorners(temp, chessboardSize,
      cv::Mat(chessboardCornersOrig), true);
#endif

  displayResult("temp", temp, true);
#if 0
  {
    int r = 1;
    for (const auto &P : chessboardCornersOrig) {
      cv::circle(temp, P, (r * 5), cv::Scalar(200, 250, 250), 3);
      ++r;
    }
  }
#endif

  // assume that we could esimate board size in pixel using two leftmost points
  // at the bottom of the chessboard
  auto twoPoints = getTwoBottomLeftPoints(orderChessboardCorners(
      chessboardCornersTemp, chessboardSize));

  cv::Point2f blp = twoPoints.first;
  cv::Point2f blpn = twoPoints.second;
  float squareSize = norm(blp - blpn);
#if 0
  cv::circle(temp, blp, 30, cv::Scalar(0, 255, 0), 10);
  cv::circle(temp, blpn, 50, cv::Scalar(0, 255, 0), 10);
#endif

  std::vector<cv::Point2f> targetRectangleCorners;
  // bottom left
  targetRectangleCorners.push_back(cv::Point2f(blp.x, blp.y));
  // bottom right
  targetRectangleCorners.push_back(
      cv::Point2f(blp.x + squareSize * (chessboardSize.width - 1), blp.y));
  // top right
  targetRectangleCorners.push_back(
    cv::Point2f(blp.x + squareSize * (chessboardSize.width - 1),
    blp.y - squareSize * (chessboardSize.height - 1)));
  // top left
  targetRectangleCorners.push_back(
    cv::Point2f(blp.x, blp.y - squareSize * (chessboardSize.height - 1)));

  for (int i = 0; i < 4; ++i) {
    cv::line(temp, targetRectangleCorners[i],
        targetRectangleCorners[(i + 1) % 4], cv::Scalar(255, 0, 0), 5 + 2 * i);
#if 0
    cv::circle(temp, targetRectangleCorners[i], 5 * (i + 1),
        cv::Scalar(255, 255, 0), 5 + 2 * i);
#endif
  }

  std::vector<cv::Point2f> currentRectangleCorners =
      extractCorners(orderChessboardCorners(chessboardCornersTemp, chessboardSize));
  chessboardCornersOrig = currentRectangleCorners;

  for (int i = 0; i < 4; ++i) {
    cv::line(temp, currentRectangleCorners[i],
        currentRectangleCorners[(i + 1) % 4],
        cv::Scalar(0, 0, 255), 5 + 2 * i);
#if 0
    cv::circle(temp, currentRectangleCorners[i], 10 * (i + 1),
        cv::Scalar(0, 255, 255), 5 + 2 * i);
#endif
  }

  displayResult("temp", temp, true);
  //cv::imwrite("temp.jpg", temp);

  cv::Mat H;
  cv::Size shift;
  computeHomography(currentRectangleCorners, targetRectangleCorners,
    cv::Size(temp.cols, temp.rows), H, shift, imageCorners);

  corners_info_t ic(imageCorners);
  cv::warpPerspective(temp, result, H, cv::Size(ic.width, ic.height));
  chessboardCorners = targetRectangleCorners;
  for (auto &P : chessboardCorners) {
    P.x += shift.width;
    P.y += shift.height;
  }

  return true;
}

void computeHomography(const std::vector<cv::Point2f> &from,
    const std::vector<cv::Point2f> &to, const cv::Size &size_from,
    cv::Mat &H, cv::Size &shift, std::vector<cv::Point2f> &imageCorners) {
  // find preliminary homography matrix
  cv::Mat preH = cv::findHomography(cv::Mat(from), cv::Mat(to), CV_RANSAC);

  std::vector<cv::Point2f> currentImageCorners = extractCorners(size_from);
  cv::Mat temp;
  cv::perspectiveTransform(cv::Mat(currentImageCorners), temp, preH);
  std::vector<cv::Point2f> newImageCorners = (std::vector<cv::Point2f>)temp;

  // apply offsets
  corners_info_t ci(newImageCorners);
  shift = cv::Size(ci.minX < 0 ? fabs(ci.minX) : 0, ci.minY < 0 ? fabs(ci.minY) : 0);
  auto to_ = to;
  for (auto &P : to_) {
    P.x += shift.width;
    P.y += shift.height;
  }

  for (auto &P : newImageCorners) {
    P.x += shift.width;
    P.y += shift.height;
  }

  imageCorners = newImageCorners;

  // recalculate homography accounting offsets
  H = cv::findHomography(cv::Mat(from), cv::Mat(to_), CV_RANSAC);
}