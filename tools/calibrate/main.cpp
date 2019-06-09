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

enum class UState {
  DELAY,
  NOT_STARTED,
  GATHERING_DATA,
  CALIBRATING,
  CALIBRATED,
  ACCEPTED
};

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
  vector<Mat> cameraMatrix(opts.file_paths.size());
  vector<Mat> distCoeffs(opts.file_paths.size());

  vector<Mat> images(opts.file_paths.size());
  vector<VideoCapture> videos(opts.file_paths.size());
  Size chessboardSize(opts.board_width, opts.board_height);

  if (!opts.video) {
    for (size_t i = 0; i < opts.file_paths.size(); ++i) {
      images[i] = imread(opts.file_paths[i]);
    }
  } else {
    for (size_t i = 0; i < opts.file_paths.size(); ++i) {
      videos[i].open(opts.file_paths[i]);
      if (!videos[i].isOpened()) {
        cout << "Failed to open file " << opts.file_paths[i] << endl;
        return 3;
      }
    }

    // Step #1. Remove distortion
    for (size_t i = 0; i < opts.file_paths.size(); ++i) {
      VideoCapture &video = videos[i];

      UState state = UState::NOT_STARTED;
      vector<Point3f> obj;
      for(int j = 0; j < opts.board_width * opts.board_height; j++)
          // TODO: squareSize
          obj.push_back(Point3f((j / opts.board_width) * 100,
              (j % opts.board_width) * 100, 0.0f));
      vector<vector<Point3f>> object_points(opts.number_of_frames, obj);
      vector<vector<Point2f>> image_points;
      Scalar color(0, 0, 0);
      string text;
      int delay = 0;
      UState next_state = UState::NOT_STARTED;
      auto last_time = chrono::steady_clock::now();
      Mat frame;
      bool calibrated = false;

      // Main loop for undistortion
      while (!calibrated) {
        video >> frame;

        switch (state) {
          case UState::DELAY: {
            --delay;
            if (delay <= 0) {
              state = next_state;
            }

            break;
          }
          case UState::NOT_STARTED: {
            image_points.clear();
            color = Scalar(0, 0, 0);
            text = "Press 's' to start gathering data";
            break;
          }
          case UState::GATHERING_DATA: {
            if (image_points.size() == opts.number_of_frames) {
              state = UState::CALIBRATING;
              break;
            }

            auto time = chrono::steady_clock::now();
            color = Scalar(0, 0, 255); // red
            if (findChessboardCorners(frame, chessboardSize,
                chessboard_corners_orig[i])) {
              color = Scalar(0, 255, 255); // yellow
              chrono::duration<double, milli> elapsed = time - last_time;
              if (elapsed.count() >= opts.delay) {
                image_points.push_back(chessboard_corners_orig[i]);
                color = Scalar(0, 255, 0); // green
                last_time = time;
              }
            }

            text = "Gathering data: " + to_string(image_points.size()) + "/" +
                to_string(opts.number_of_frames);
            break;
          }
          case UState::CALIBRATING: {
            cameraMatrix[i] = Mat::eye(3, 3, CV_64F);
            distCoeffs[i] = Mat::zeros(8, 1, CV_64F);
            vector<Mat> rvecs;
            vector<Mat> tvecs;
            calibrateCamera(object_points, image_points, frame.size(),
                cameraMatrix[i], distCoeffs[i], rvecs, tvecs,
                CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);
            if (checkRange(cameraMatrix[i]) && checkRange(distCoeffs[i])) {
              color = Scalar(0, 255, 0); // green
              text = "Calibrated sucessfully";
              delay = 30;
              next_state = UState::CALIBRATED;
            } else {
              color = Scalar(0, 0, 255); // red
              text = "Failed to calibrate. Re-starting";
              delay = 30;
              next_state = UState::NOT_STARTED;
            }

            state = UState::DELAY;
            break;
          }
          case UState::CALIBRATED: {
            Mat undistorted;
            undistort(frame, undistorted, cameraMatrix[i], distCoeffs[i]);
            frame = undistorted;
            color = Scalar(0, 0, 0);
            text = "Undistorted. Press 'y' to continue, 'n' to restart";
            break;
          }
          case UState::ACCEPTED: {
            // swith to a next camera
            calibrated = true;

            break;
          }
        }

        if (Scalar(0, 0, 0) != color) {
          rectangle(frame, Point2f(0, 0), Point2f(frame.cols, frame.rows),
              color, 8);
        }
        putText(frame, text, Point2f(10, 30), FONT_HERSHEY_SIMPLEX, 1.0,
            Scalar(255, 0 , 0));
        displayResult("Frame from camera " + to_string(i), frame);
        char key = waitKey(30);
        if (key == 'y') {
          state = UState::DELAY;
          next_state = UState::ACCEPTED;
          delay = 30;
          color = Scalar(0, 0, 0);
          text = "Switching to a next camera";
        } else if (key == 'n') {
          state = UState::DELAY;
          next_state = UState::NOT_STARTED;
          text = "Re-starting";
          color = Scalar(0, 0, 0);
          delay = 30;
        } else if (state == UState::NOT_STARTED && key == 's') {
          state = UState::GATHERING_DATA;
        }
      }
    } // End of step 1: disctorion was removed

    // Step 2: Find good frames for alignment and stitching
    for (size_t i = 0; i < opts.file_paths.size() - 1; ++i) {
      Mat frameL, frameR;
      Mat tL, tR;
      VideoCapture &videoL = videos[i], &videoR = videos[i + 1];
      bool found_good_frame = false;
      while (!found_good_frame) {
        videoL >> tL;
        videoR >> tR;
        undistort(tL, frameL, cameraMatrix[i], distCoeffs[i]);
        undistort(tR, frameR, cameraMatrix[i + 1], distCoeffs[i + 1]);

        bool chessboardL = findChessboardCorners(frameL, chessboardSize,
            chessboard_corners_orig[i]);
        bool chessboardR = findChessboardCorners(frameR, chessboardSize,
            chessboard_corners_orig[i + 1]);

        // red or orange
        Scalar color = chessboardL ? Scalar(0, 0, 255) : Scalar(0, 165, 255);

        if (chessboardL && chessboardR) {
          float angle = angleToHorizon(chessboard_corners_orig[i],
              chessboardSize);
          if (angle < 1) {
            color = Scalar(0, 255, 0); // green
            found_good_frame = true;
            images[i] = frameL;
            images[i + 1] = frameR;
          } else if (angle < 10) {
            color = Scalar(0, 255, 255); // yellow
          }
          putText(frameL, "Angle: " + std::to_string(angle), Point2f(10, 30),
              FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0 , 0));
        }
        rectangle(frameL, Point2f(0, 0), Point2f(frameL.cols, frameL.rows),
            color, 8);
        rectangle(frameR, Point2f(0, 0), Point2f(frameR.cols, frameR.rows),
            color, 8);
        displayResult("Frame from camera " + std::to_string(i), frameL);
        displayResult("Frame from camera " + std::to_string(i + 1), frameR);
        cv::waitKey(30);
      }
    }
  }

  for (size_t i = 0; i < opts.file_paths.size(); ++i) {
    Mat image = images[i];

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

  fs << "video" << opts.video;
  fs << "file_paths" << "[";
  for (int i = 0; i < opts.file_paths.size(); ++i) {
    fs << opts.file_paths[i];
  }
  fs << "]";
  fs << "result_size" << result_size;
  fs << "H" << "[";
  for (int i = 0; i < opts.file_paths.size(); ++i) {
    fs << H[i];
  }
  fs << "]";
  fs << "cameraMatrix" << "[";
  for (size_t i = 0; i < opts.file_paths.size(); ++i) {
    fs << cameraMatrix[i];
  }
  fs << "]";
  fs << "distCoeffs" << "[";
  for (size_t i = 0; i < opts.file_paths.size(); ++i) {
    fs << distCoeffs[i];
  }
  fs << "]";

  fs.release();

  return 0;
}
