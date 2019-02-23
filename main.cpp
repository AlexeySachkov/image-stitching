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
  vector<Point2f> secondCorners = extractCorners(result2);
  Mat tcorners;
  perspectiveTransform(Mat(secondCorners), tcorners, preH);
  vector<Point2f> newSecondCorners = (vector<Point2f>(tcorners));
  corners_info_t ci(newSecondCorners);

  vector<Point2f> firstCorners = extractCorners(result1);
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
