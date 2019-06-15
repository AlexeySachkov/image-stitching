#include "utils.hpp"
#include "opts.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cmath>
#include <algorithm>

extern command_line_opts opts;

namespace {

const cv::Point2f &getPoint(const std::vector<cv::Point2f> &points,
    size_t stride, size_t y, size_t x) {
  return points[y * stride + x];
}

const cv::Size calculateSizeForDisplaying(const cv::Size &originalSize,
    const cv::Size &screenSize = cv::Size(1920, 1080)) {
  // To make looking at several images eaiser, each of
  // them should not occupy more than half of the screen.
  // For height this restriction is relaxed
  float targetH = (float)screenSize.height * 0.7;
  float targetW = (float)screenSize.width * 0.7;

  float hRatio = originalSize.height / targetH;
  float wRatio = originalSize.width / targetW;

  float ratio = fmax(hRatio, wRatio);
  return cv::Size((int)round(originalSize.width / ratio),
      (int)round(originalSize.height / ratio));
}

std::vector<std::vector<cv::Point2f>> transpose(
    const std::vector<std::vector<cv::Point2f>> &src) {
  std::vector<std::vector<cv::Point2f>> result(src.front().size());
  for (size_t i = 0; i < result.size(); ++i) {
    for (size_t j = 0; j < src.size(); ++j) {
      result[i].push_back(src[j][i]);
    }
  }

  return result;
}

std::vector<std::vector<cv::Point2f>> interpretAsByColumn2dArray(
    const std::vector<cv::Point2f> &src, const cv::Size &size) {
  std::vector<std::vector<cv::Point2f>> result(size.height);
  assert(src.size() == size.height * size.width && "Invalid input");
  size_t index = 0;
  for (int i = 0; i < size.width; ++i) {
    for (size_t j = 0; j < result.size(); ++j) {
      result[j].push_back(src[index]);
      ++index;
    }
  }

  return result;
}

std::vector<std::vector<cv::Point2f>> interpretAsByRow2dArray(
    const std::vector<cv::Point2f> &src, const cv::Size &size) {
  std::vector<std::vector<cv::Point2f>> result(size.height);
  assert(src.size() == size.height * size.width && "Invalid input");
  size_t index = 0;
  for (size_t i = 0; i < result.size(); ++i) {
    for (int j = 0; j < size.width; ++j) {
      result[i].push_back(src[index]);
      ++index;
    }
  }

  return result;
}

enum class Component : size_t {
  X = 0, Y = 1
};

bool areThereNMonotonousPoints(const std::vector<cv::Point2f> &p,
    const int N, const Component c) {
  int numDec = 0;
  assert(N + 1 < p.size() && "Invalid input");
  for (size_t i = 1; i < N + 1; ++i) {
    const auto cur = (cv::Vec<float, 2>)p[i];
    const auto prev = (cv::Vec<float, 2>)p[i - 1];
    if (cur[static_cast<size_t>(c)] > prev[static_cast<size_t>(c)]) {
      ++numDec;
    } else {
      break;
    }
  }
  int numInc = 0;
  for (size_t i = 1; i < N + 1; ++i) {
    const auto cur = (cv::Vec<float, 2>)p[i];
    const auto prev = (cv::Vec<float, 2>)p[i - 1];
    if (cur[static_cast<size_t>(c)] < prev[static_cast<size_t>(c)]) {
      ++numInc;
    } else {
      break;
    }
  }

  return (N - 1 == numDec && 0 == numInc) || (N - 1 == numInc && 0 == numDec);
}

void getPointsOrientation(const std::vector<cv::Point2f> &p,
    const cv::Size &bs, bool &orderedByRows, bool &transposed) {
  // let's assume that points are ordered by rows and not transposed:
  if (areThereNMonotonousPoints(p, bs.width, Component::X)) {
    orderedByRows = true;
    transposed = false;
    return;
  }

  // let's assume that points are ordered by columns and not transposed:
  if (areThereNMonotonousPoints(p, bs.height, Component::Y)) {
    orderedByRows = false;
    transposed = false;
    return;
  }

  // let's assume that points are ordered by columns and transposed:
  if (areThereNMonotonousPoints(p, bs.width, Component::Y)) {
    orderedByRows = false;
    transposed = true;
    return;
  }

  // let's assume that points are ordered by rows and transposed:
  if (areThereNMonotonousPoints(p, bs.height, Component::X)) {
    orderedByRows = true;
    transposed = true;
    return;
  }

  assert(false && "Unexpected order of points");
}

}

std::vector<std::vector<cv::Point2f>> orderChessboardCorners(
    const std::vector<cv::Point2f> &chessboardCorners,
    const cv::Size &boardSize) {
  bool isByRow = false;
  bool isTransposed = false;
  getPointsOrientation(chessboardCorners, boardSize, isByRow, isTransposed);

  std::vector<std::vector<cv::Point2f>> result;
  cv::Size tSize = isTransposed ? cv::Size(boardSize.height, boardSize.width)
      : boardSize;

  if (isByRow) {
    result = interpretAsByRow2dArray(chessboardCorners, tSize);
  } else {
    result = interpretAsByColumn2dArray(chessboardCorners, tSize);
  }

  if (isTransposed) {
    result = transpose(result);
  }

  // Now result is by-row 2d array of points

  bool needToReverseColumns = false;
  bool needToReverseRows = false;

  if (isTransposed) {
    // the top row should have the biggest X value, because it was the
    // rightmost column before transpose
    if (result[0][0].x < result[1][0].x)
      needToReverseRows = true;
    // the leftmost column should have the smallest Y value, because it was
    // the top top before transpose
    if (result[0][0].y > result[0][1].y)
      needToReverseColumns = true;
  } else {
    if (result[0][0].y > result[1][0].y)
      needToReverseRows = true;
    if (result[0][0].x > result[0][1].x)
      needToReverseColumns = true;
  }

  if (needToReverseColumns) {
    for (auto &row : result) {
      std::reverse(row.begin(), row.end());
    }
  }

  if (needToReverseRows) {
    std::reverse(result.begin(), result.end());
  }

  return result;
}

std::pair<cv::Point2f, cv::Point2f> getTwoBottomLeftPoints(
    const std::vector<std::vector<cv::Point2f>> &points) {
  return std::make_pair(points.back()[0], points.back()[1]);
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

bool findChessboardCorners(const cv::Mat &image,
    const cv::Size &chessboardSize,
    std::vector<cv::Point2f> &chessboardCorners) {
  // search for chessboard corners
  if (!cv::findChessboardCorners(image, chessboardSize, chessboardCorners,
      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |
      CV_CALIB_CB_NORMALIZE_IMAGE))
    return false;

  // optimize results
  cv::Mat viewGray;
  cv::cvtColor(image, viewGray, CV_BGR2GRAY);
  cv::cornerSubPix(viewGray, chessboardCorners, cv::Size(11, 11),
      cv::Size(-1, -1),
      cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  return true;
}

float angleToHorizon(const std::vector<cv::Point2f> &chessboardCorners,
    const cv::Size &chessboardSize) {
  auto twoPoints = getTwoBottomLeftPoints(orderChessboardCorners(
    chessboardCorners, chessboardSize));
  auto A = twoPoints.first;
  auto B = twoPoints.second;
  cv::Point2f C(B.x, A.y);

  float BC = fabs(B.y - A.y);
  float AB = sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));

  float S = BC / AB;
  float R = asin(S);

  const float PI = 3.1415;
  R = R*180/PI;

  return R;
}

bool projectToTheFloor(const cv::Mat &image, const cv::Size &chessboardSize,
    cv::Mat &result, std::vector<cv::Point2f> &chessboardCornersOrig,
    std::vector<cv::Point2f> &chessboardCorners,
    std::vector<cv::Point2f> &imageCorners) {
  std::vector<cv::Point2f> chessboardCornersTemp;
  if (!findChessboardCorners(image, chessboardSize, chessboardCornersTemp)) {
    return false;
  }

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
    std::vector<std::vector<cv::Point2f>> ordered = orderChessboardCorners(
        chessboardCornersTemp, chessboardSize);
    for (size_t i = 0; i < ordered.size(); ++i) {
      for (size_t j = 0; j < ordered[i].size(); ++j) {
        const cv::Point2f &P = ordered[i][j];
        cv::circle(temp, P, (r * 5), cv::Scalar(200, 250, 250), 3);
        ++r;
      }
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
