#include "utils.hpp"

#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

const Size bSize(5, 4);
const int squareSize = 100;

struct corners_info_t {
  corners_info_t() = delete;
  corners_info_t(const vector<Point2f> &corners) {
    minX = fmin(fmin(corners[0].x, corners[1].x), fmin(corners[2].x, corners[3].x));
    maxX = fmax(fmax(corners[0].x, corners[1].x), fmax(corners[2].x, corners[3].x));
    minY = fmin(fmin(corners[0].y, corners[1].y), fmin(corners[2].y, corners[3].y));
    maxY = fmax(fmax(corners[0].y, corners[1].y), fmax(corners[2].y, corners[3].y));
    width = maxX - minX;
    height = maxY - minY;
  }

  void print() {
    cout << "X: " << minX << " " << maxX << endl;
    cout << "Y: " << minY << " " << maxY << endl;
    cout << "H x W: " << height << " " << width << endl;
  }

  float minX, maxX, minY, maxY;
  float width, height;
};

void getImageCorners(const Mat &image, vector<Point2f> &corners) {
  corners.clear();
  corners.push_back(Point2f(0, image.rows)); // bottom left
  corners.push_back(Point2f(image.cols, image.rows)); // bottom right
  corners.push_back(Point2f(image.cols, 0)); // top right
  corners.push_back(Point2f(0, 0)); // top left
}

bool projectToTheFloor(const Mat &image, const Size &boardSize, Mat &result, vector<Point2f> &rectangle, vector<Point2f> &corners) {
  // search for chessboard corners
  vector<Point2f> chessboardCorners;
  if (!findChessboardCorners(image, boardSize, chessboardCorners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE))
    return false;

  // optimize results
  Mat viewGray;
  cvtColor(image, viewGray, CV_BGR2GRAY);
  cornerSubPix(viewGray, chessboardCorners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

  Mat temp;
  image.copyTo(temp);
  drawChessboardCorners(temp, boardSize, Mat(chessboardCorners), true);
  displayResult("temp", temp);
  waitKey();
  {
    int r = 1;
    for (const auto &P : chessboardCorners) {
      circle(temp, P, (r * 5), Scalar(200, 250, 250), 3);
      ++r;
    }
  }

  pair<Point2f, Point2f> twoPoints = getTwoBottomLeftPoints(orderChessboardCorners(chessboardCorners, boardSize));

  // assume that we could esimate board size in pixel using two leftmost points at the bottom of the chessboard
  Point2f blp = twoPoints.first;
  Point2f blpn = twoPoints.second;
  float squareSize = norm(blp - blpn);
  circle(temp, blp, 30, Scalar(0, 255, 0), 10);
  circle(temp, blpn, 50, Scalar(0, 255, 0), 10);

  vector<Point2f> targetRectangleCorners;
  targetRectangleCorners.push_back(Point2f(blp.x, blp.y)); // bottom left
  targetRectangleCorners.push_back(Point2f(blp.x + squareSize * (boardSize.width - 1), blp.y)); // bottom right
  targetRectangleCorners.push_back(Point2f(blp.x + squareSize * (boardSize.width - 1), blp.y - squareSize * (boardSize.height - 1))); // top right
  targetRectangleCorners.push_back(Point2f(blp.x, blp.y - squareSize * (boardSize.height - 1))); // top left
  cout << "boardSize: " << boardSize << endl;
  cout << "target: " << targetRectangleCorners << endl;

  for (int i = 0; i < 4; ++i) {
    line(temp, targetRectangleCorners[i], targetRectangleCorners[(i + 1) % 4], Scalar(255, 0, 0), 5 + 2 * i);
    circle(temp, targetRectangleCorners[i], 5 * (i + 1), Scalar(255, 255, 0), 5 + 2 * i);
  }

  vector<Point2f> currentRectrangleCorners = extractCorners(orderChessboardCorners(chessboardCorners, boardSize));
  
  for (int i = 0; i < 4; ++i) {
    line(temp, currentRectrangleCorners[i], currentRectrangleCorners[(i + 1) % 4], Scalar(0, 0, 255), 5 + 2 * i);
    circle(temp, currentRectrangleCorners[i], 10 * (i + 1), Scalar(0, 255, 255), 5 + 2 * i);
  }

  displayResult("temp", temp);
  waitKey();
  imwrite("temp.jpg", temp);

  // find preliminary homography matrix
  Mat preH = findHomography(Mat(currentRectrangleCorners), Mat(targetRectangleCorners), CV_RANSAC);

  vector<Point2f> currentCorners;
  getImageCorners(image, currentCorners);

  Mat tcorners;
  perspectiveTransform(Mat(currentCorners), tcorners, preH);
  vector<Point2f> newCorners = (vector<Point2f>)tcorners;

  corners_info_t ci(newCorners);
  float newWidth = ci.width;
  float newHeight = ci.height;

  // apply offsets
  for (auto &P : targetRectangleCorners) {
    P.x += fabs(ci.minX);
    P.y += fabs(ci.minY);
  }
  for (auto &P : newCorners) {
    P.x += fabs(ci.minX);
    P.y += fabs(ci.minY);
  }

  // recalculate homography accounting offsets
  Mat H = findHomography(Mat(currentRectrangleCorners), Mat(targetRectangleCorners), CV_RANSAC);

  warpPerspective(temp, result, H, Size(newWidth, newHeight));
  corners = newCorners;
  rectangle = targetRectangleCorners;

  return true;
}

int main()
{
  vector<Point2f> corners1;
  vector<Point2f> rectangle1;
  Mat result1;
  Mat image1 = imread("IMG_20190207_124236 - Copy.jpg");

  if (!projectToTheFloor(image1, Size(3, 4), result1, rectangle1, corners1)) {
    cout << "Failed to handle first image!" << endl;
    return -1;
  }

  displayResult("image1", image1);
  displayResult("result1", result1);
  waitKey();
  imwrite("result1.jpg", result1);

  vector<Point2f> corners2;
  vector<Point2f> rectangle2;
  Mat result2;
  Mat image2 = imread("IMG_20190207_124244 - Copy.jpg");

  if (!projectToTheFloor(image2, Size(3, 4), result2, rectangle2, corners2)) {
    cout << "Failed to handle second image!" << endl;
    return -1;
  }

  displayResult("image2", image2);
  displayResult("result2", result2);
  waitKey();
  imwrite("result2.jpg", result2);

  vector<Point2f> rotatedRect2;
  if (false) {
    rotatedRect2.push_back(rectangle2[3]);
    rotatedRect2.push_back(rectangle2[0]);
    rotatedRect2.push_back(rectangle2[1]);
    rotatedRect2.push_back(rectangle2[2]);
  } else {
    rotatedRect2 = rectangle2;
  }

  Mat preH = findHomography(Mat(rotatedRect2), Mat(rectangle1), CV_RANSAC);
  vector<Point2f> secondCorners;
  getImageCorners(result2, secondCorners);
  Mat tcorners;
  perspectiveTransform(Mat(secondCorners), tcorners, preH);
  vector<Point2f> newSecondCorners = (vector<Point2f>(tcorners));
  corners_info_t ci(newSecondCorners);

  vector<Point2f> firstCorners;
  getImageCorners(result1, firstCorners);
  corners_info_t fci(firstCorners);

  float minX = fmin(ci.minX, fci.minX);
  float maxX = fmax(ci.maxX, fci.maxX);
  float minY = fmin(ci.minY, fci.minY);
  float maxY = fmax(ci.maxY, fci.maxY);

  float height = maxY - minY;
  float width = maxX - minX;

  Mat result(Size(width, height), image1.type());

  Rect roi1(Point(fabs(minX), fabs(minY)), Size(fci.width, fci.height));
  Rect roi2(Point(0, 0), Size(ci.width, ci.height));

  vector<Point2f> shiftedRect1 = rectangle1;
  for (auto &P : shiftedRect1) {
    P.x += fabs(minX);
    P.y += fabs(minY);
  }

  Mat H = findHomography(Mat(rotatedRect2), Mat(shiftedRect1), CV_RANSAC);

  Mat rotated2;
  warpPerspective(result2, rotated2, H, Size(ci.width, ci.height));

  Mat destRoi1 = result(roi1);
  Mat destRoi2 = result(roi2);

  Mat mask1(Size(fci.width, fci.height), image1.type());
  vector<Point> cm1;
  for (auto &P : corners1) {
    cm1.push_back(P);
  }
  cout << "cm1: " << cm1 << endl;
  fillConvexPoly(mask1, cm1, Scalar(255, 255, 255));

  Mat mask2(Size(ci.width, ci.height), image2.type());
  vector<Point2f> rotatedCorners2;
  if (false) {
    rotatedCorners2.push_back(corners2[3]);
    rotatedCorners2.push_back(corners2[0]);
    rotatedCorners2.push_back(corners2[1]);
    rotatedCorners2.push_back(corners2[2]);
  } else {
    rotatedCorners2 = corners2;
  }

  Mat tcorners2;
  perspectiveTransform(Mat(rotatedCorners2), tcorners2, H);
  vector<Point2f> newCorners2 = (vector<Point2f>)tcorners2;
  vector<Point> cm2;
  for (auto &P : newCorners2) {
    cm2.push_back(P);
  }
  cout << "cm2: " << cm2 << endl;
  fillConvexPoly(mask2, cm2, Scalar(255, 255, 255));

  result1.copyTo(destRoi1, mask1);
  rotated2.copyTo(destRoi2, mask2);

  displayResult("final", result);
  waitKey();
  imwrite("result.jpg", result);

  return 0;
}
