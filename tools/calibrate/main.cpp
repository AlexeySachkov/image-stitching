#include "Debug.hpp"
#include "utils.hpp"
#include "opts.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"

#include <iostream>
#include <vector>
#include <chrono>

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
  vector<Mat> U(opts.file_paths.size());

  vector<Mat> images(opts.file_paths.size());
  if (!opts.video) {
    for (size_t i = 0; i < opts.file_paths.size(); ++i) {
      Mat image = imread(opts.file_paths[i]);
    }
  } else {
    for (size_t i = 0; i < opts.file_paths.size(); ++i) {
      VideoCapture video(opts.file_paths[i]);
      if (!video.isOpened()) {
        cout << "Failed to open file " << opts.file_paths[i] << endl;
        return 3;
      }

      Mat frame;
      Size chessboardSize(opts.board_width, opts.board_height);
      vector<Point3f> obj;
      for(int j = 0; j < opts.board_width * opts.board_height; j++)
          // TODO: squareSize
          obj.push_back(Point3f((j / opts.board_width) * 100,
              (j % opts.board_width) * 100, 0.0f));

      bool undistorted = false;
      bool started = false;
      auto last_time = chrono::steady_clock::now();
      unsigned f = 0;
      while (!undistorted) {
        vector<vector<Point2f>> image_points;
        vector<vector<Point3f>> object_points;

        while (image_points.size() < opts.number_of_frames) {
          video >> frame;
          ++f;

          if (started) {
            auto time = chrono::steady_clock::now();
            Scalar color = Scalar(0, 0, 255); // red
            if (findChessboardCorners(frame, chessboardSize,
                chessboard_corners_orig[i])) {
              color = Scalar(0, 255, 255); // yellow
              if (chrono::duration<double, std::milli>(time - last_time).count() >= opts.delay) {
                image_points.push_back(chessboard_corners_orig[i]);
                object_points.push_back(obj);
                color = Scalar(0, 255, 0); // green
                last_time = time;
              }
            }
            rectangle(frame, Point2f(0, 0), Point2f(frame.cols, frame.rows), color, 8);
            putText(frame, "Undistort: " + std::to_string(image_points.size())
                + "/" + std::to_string(opts.number_of_frames), Point2f(10, 30),
                FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0 , 0));
          } else {
            putText(frame, "Undistort: press s to start", Point2f(10, 30),
                FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0 , 0));
          }
          displayResult("Frame from camera " + std::to_string(i), frame);
          auto key = waitKey(30);
          if (!started && key == 's') {
            started = true;
          }
        }

        Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
        Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
        vector<Mat> rvecs;
        vector<Mat> tvecs;
        calibrateCamera(object_points, image_points, frame.size(),
            cameraMatrix, distCoeffs, rvecs, tvecs,
            CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);
        bool calibrated_good = checkRange(cameraMatrix) && checkRange(distCoeffs);
        // green or red
        Scalar color = calibrated_good ? Scalar(0, 255, 0) : Scalar(0, 0, 255);

        while (true) {
          video >> frame;
          Mat undistorted;
          rectangle(frame, Point2f(0, 0), Point2f(frame.cols, frame.rows), color, 8);
          undistort(frame, undistorted, cameraMatrix, distCoeffs);
          putText(undistorted,
              "Undistorted. Presss 'y' to confirm, 'n' to restart",
              Point2f(10, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0 , 0));
          displayResult("Frame from camera " + std::to_string(i), undistorted);
          auto key = waitKey(30);
          if (key == 'y') {
            undistorted = true;
            break;
          } else if (key == 'n') {
            break;
          }
        }
      }

      bool found_good_frame = false;
      while (!found_good_frame) {
        video >> frame;
        Scalar color = Scalar(0, 0, 255); // red
        if (findChessboardCorners(frame, Size(opts.board_width, opts.board_height),
            chessboard_corners_orig[i])) {
          float angle = angleToHorizon(chessboard_corners_orig[i], Size(opts.board_width, opts.board_height));
          if (angle < 1) {
            color = Scalar(0, 255, 0); // green
          } else if (angle < 10) {
            color = Scalar(0, 255, 255); // yellow
          }
          putText(frame, "Angle: " + std::to_string(angle), Point2f(10, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0 , 0));
        }
        rectangle(frame, Point2f(0, 0), Point2f(frame.cols, frame.rows), color, 8);
        displayResult("Frame from camera " + std::to_string(i), frame);
        cv::waitKey(30);
      }
    }

    return 0;
  }

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
