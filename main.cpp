#include "utils.hpp"
#include "opts.hpp"

#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

const Size bSize(5, 4);
const int squareSize = 100;

extern command_line_opts opts;

int main(int argc, char *argv[])
{
  if (!parse_command_line_opts(argc, argv)) {
    cout << "Usage: " << argv[0] << " /path/to/img1.jpg /path/to/img2.jpg";
    return 1;
  }

  // prepare base image
  vector<Point2f> rectangle_base;
  Mat base;
  Mat image = imread(opts.file_paths.front());

  if (!projectToTheFloor(image, Size(3, 4), base, rectangle_base)) {
    cout << "Failed to handle base image!" << endl;
    return -1;
  }

  displayResult("image", image);
  displayResult("base", base, true);
  //imwrite("base.jpg", base);

  // Algorithm:
  // 1. Prepare base image
  // 2. Stich a next image to the base image
  // 3. Use stitched image as base, repeat from step 1
  //
  // How to stitch:
  // 1. Project to the floor
  // 2. Find homography using chessboard corners
  // 3. Use warpPerspective

  for (size_t i = 1; i < opts.file_paths.size(); ++i) {
    // stitch the next image to the base image
    vector<Point2f> rectangle;
    Mat next_projected;
    Mat next_image = imread(opts.file_paths[i]);

    if (!projectToTheFloor(next_image, Size(3, 4), next_projected, rectangle)) {
      cout << "Failed to handle second image!" << endl;
      return -1;
    }

    displayResult("next_image", next_image);
    displayResult("next_projected", next_projected, true);
    //imwrite("next_projected.jpg", next_projected);

    // TODO: automatically detect rotation
    vector<Point2f> rotatedRect2;
    if (false) {
      rotatedRect2.push_back(rectangle[3]);
      rotatedRect2.push_back(rectangle[0]);
      rotatedRect2.push_back(rectangle[1]);
      rotatedRect2.push_back(rectangle[2]);
    } else {
      rotatedRect2 = rectangle;
    }

    Mat preH = findHomography(Mat(rotatedRect2), Mat(rectangle_base), CV_RANSAC);
    vector<Point2f> secondCorners = extractCorners(next_projected);
    Mat tcorners;
    perspectiveTransform(Mat(secondCorners), tcorners, preH);
    vector<Point2f> newSecondCorners = (vector<Point2f>(tcorners));
    corners_info_t ci(newSecondCorners);

    vector<Point2f> firstCorners = extractCorners(base);
    corners_info_t fci(firstCorners);

    float minX = fmin(ci.minX, fci.minX);
    float maxX = fmax(ci.maxX, fci.maxX);
    float minY = fmin(ci.minY, fci.minY);
    float maxY = fmax(ci.maxY, fci.maxY);

    float height = maxY - minY;
    float width = maxX - minX;

    Mat merged(Size(width, height), base.type());

    Rect roi1(Point(fabs(minX), fabs(minY)), Size(fci.width, fci.height));
    Rect roi2(Point(0, 0), Size(ci.width, ci.height));

    vector<Point2f> shiftedRect1 = rectangle_base;
    for (auto &P : shiftedRect1) {
      P.x += fabs(minX);
      P.y += fabs(minY);
    }

    Mat H = findHomography(Mat(rotatedRect2), Mat(shiftedRect1), CV_RANSAC);

    Mat rotated2;
    warpPerspective(next_projected, rotated2, H, Size(ci.width, ci.height));

    Mat destRoi1 = merged(roi1);
    Mat destRoi2 = merged(roi2);

    Mat mask1(Size(fci.width, fci.height), base.type());
    vector<Point> cm1;
    for (auto &P : extractCorners(base)) {
      cm1.push_back(P);
    }
    fillConvexPoly(mask1, cm1, Scalar(255, 255, 255));

    Mat mask2(Size(ci.width, ci.height), base.type());
    vector<Point2f> rotatedCorners2;
    vector<Point2f> corners = extractCorners(next_projected);
    if (false) {
      rotatedCorners2.push_back(corners[3]);
      rotatedCorners2.push_back(corners[0]);
      rotatedCorners2.push_back(corners[1]);
      rotatedCorners2.push_back(corners[2]);
    } else {
      rotatedCorners2 = corners;
    }

    Mat tcorners2;
    perspectiveTransform(Mat(rotatedCorners2), tcorners2, H);
    vector<Point2f> newCorners2 = (vector<Point2f>)tcorners2;
    vector<Point> cm2;
    for (auto &P : newCorners2) {
      cm2.push_back(P);
    }
    fillConvexPoly(mask2, cm2, Scalar(255, 255, 255));

    base.copyTo(destRoi1, mask1);
    rotated2.copyTo(destRoi2, mask2);

    displayResult("final", merged, true);
    //imwrite("merged.jpg", merged);

    // Update the base image
    base = merged;
    rectangle_base = rectangle;
  }

  return 0;
}
