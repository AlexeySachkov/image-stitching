#include "Debug.hpp"
#include "utils.hpp"
#include "opts.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"

#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

extern command_line_opts opts;

int main(int argc, char *argv[])
{
  if (!parse_command_line_opts(argc, argv)) {
    cout << "Usage: " << argv[0] << " /path/to/img1.jpg /path/to/img2.jpg";
    return 1;
  }

  WITH_DEBUG(cout << "Command line arguments parsed successfully" << endl;)

  if (opts.file_paths.empty()) {
    cout << "Usage: " << argv[0] << " /path/to/img1.jpg /path/to/img2.jpg";
    return 2;
  }

  vector<Mat> projected(opts.file_paths.size());
  vector<vector<Point2f>> chessboard_corners_orig(opts.file_paths.size());
  vector<vector<Point2f>> chessboard_corners_target(opts.file_paths.size());
  vector<vector<Point2f>> image_corners_target(opts.file_paths.size());
  vector<Mat> H(opts.file_paths.size());

  for (size_t i = 0; i < opts.file_paths.size(); ++i) {
    Mat image = imread(opts.file_paths[i]);

    if (!projectToTheFloor(image, Size(opts.board_width, opts.board_height),
        projected[i], chessboard_corners_orig[i], chessboard_corners_target[i],
        image_corners_target[i])) {
      cout << "Failed to handle image #" << i + 1 << "!" << endl;
      return -1;
    }

    WITH_DEBUG(cout << "Successfully projected " << i << "-th image to the floor" << endl;)
  }

  vector<Point2f> chessboard_corners = chessboard_corners_target.front();
  Size result_size(projected.front().cols, projected.front().rows);

  for (size_t i = 1; i < opts.file_paths.size(); ++i) {
    WITH_DEBUG(
      cout << "Trying to find homography between " << endl
        << chessboard_corners_target[i] << " and " << endl
        << chessboard_corners << endl;
    )
    Mat preH = findHomography(chessboard_corners_target[i], chessboard_corners, CV_RANSAC);

    Mat temp;
    perspectiveTransform(Mat(image_corners_target[i]), temp, preH);
    vector<Point2f> image_corners = (vector<Point2f>)temp;

    float minX = image_corners.front().x;
    float minY = image_corners.front().y;
    float maxX = image_corners.front().x;
    float maxY = image_corners.front().y;

    for (size_t j = 1; j < image_corners.size(); ++j) {
      if (image_corners[j].x < minX) {
        minX = image_corners[j].x;
      } else if (image_corners[j].x > maxX) {
        maxX = image_corners[j].x;
      }
      if (image_corners[j].y < minY) {
        minY = image_corners[j].y;
      } else if (image_corners[j].y > maxY) {
        maxY = image_corners[j].y;
      }
    }

    float dx = 0, dy = 0;
    float sx = 0, sy = 0;
    if (minX < 0) {
      dx = fabs(minX);
    } else {
      sx = minX;
    }
    if (minY < 0) {
      dy = fabs(minY);
    } else {
      sy = minY;
    }

    for (int j = 0; j < chessboard_corners.size(); ++j) {
      chessboard_corners[j].x += dx;
      chessboard_corners[j].y += dy;
    }

    for (int j = 0; j <= i; ++j) {
      WITH_DEBUG(
        cout << "Trying to find homography between " << endl
          << chessboard_corners_orig[i] << " and " << endl
          << chessboard_corners << endl;
      )
      H[j] = findHomography(chessboard_corners_orig[j], chessboard_corners, CV_RANSAC);
    }

    result_size = Size(max((float)result_size.width, maxX - sx),
      max((float)result_size.height, maxY - sy));
  }

  FileStorage fs(opts.output_file, FileStorage::WRITE);

  fs << "result_size" << result_size;
  fs << "H" << "[";
  for (int i = 0; i < opts.file_paths.size(); ++i) {
    fs << H[i];
  }
  fs << "]";

  fs.release();

  return 0;
}
