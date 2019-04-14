#include "utils.hpp"
#include "opts.hpp"

#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

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
  vector<Point2f> corners_base;
  Mat base;
  Mat image = imread(opts.file_paths.front());

  if (!projectToTheFloor(image, Size(opts.board_width, opts.board_height),
      base, rectangle_base, corners_base)) {
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
    vector<Point2f> corners;
    Mat next_projected;
    Mat next_image = imread(opts.file_paths[i]);

    if (!projectToTheFloor(next_image, Size(opts.board_width, opts.board_height),
        next_projected, rectangle, corners)) {
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

    Mat H;
    Size shift;
    vector<Point2f> npNewCorners;
    computeHomography(rotatedRect2, rectangle_base, Size(base.cols, base.rows),
        H, shift, npNewCorners);

    corners_info_t bc(corners_base);
    corners_info_t nc(npNewCorners);

    Mat merged(Size(max(bc.width + shift.width, nc.width),
        max(bc.height + shift.height, nc.height)), base.type());

    Rect roi1(Point(shift.width, shift.height), Size(base.cols, base.rows));
    Rect roi2(Point(0, 0), Size(merged.cols, merged.rows));

    Mat rotated2;
    warpPerspective(next_projected, rotated2, H, Size(merged.cols, merged.rows));

    Mat destRoi1 = merged(roi1);
    Mat destRoi2 = merged(roi2);

    Mat mask2(Size(merged.cols, merged.rows), base.type());
    vector<Point2f> rotatedCorners2;
    cv::Mat temp;
    cv::perspectiveTransform(cv::Mat(corners), temp, H);
    corners = (std::vector<cv::Point2f>)temp;
    if (false) {
      rotatedCorners2.push_back(corners[3]);
      rotatedCorners2.push_back(corners[0]);
      rotatedCorners2.push_back(corners[1]);
      rotatedCorners2.push_back(corners[2]);
    } else {
      rotatedCorners2 = corners;
    }

    vector<Point> cm2;
    for (auto &P : corners) {
      cm2.push_back(P);
    }
    fillConvexPoly(mask2, cm2, Scalar(255, 255, 255));
    displayResult("mask2", mask2);
    displayResult("rotated2", rotated2, true);

    base.copyTo(destRoi1);
    rotated2.copyTo(destRoi2, mask2);

    displayResult("final", merged, true);
    //imwrite("merged.jpg", merged);

    // Update the base image
    base = merged;
    rectangle_base = rectangle;
  }

  return 0;
}
