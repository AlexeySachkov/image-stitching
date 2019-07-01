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
  vector<vector<Point2f>> chessboard_corners_orig_left(opts.file_paths.size());
  vector<vector<Point2f>> chessboard_corners_target_left(opts.file_paths.size());
  vector<vector<Point2f>> chessboard_corners_orig_right(opts.file_paths.size());
  vector<vector<Point2f>> chessboard_corners_target_right(opts.file_paths.size());
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

    if (!opts.calibrate_config.empty()) {
      FileStorage fs(opts.calibrate_config, FileStorage::READ);
      if (!fs.isOpened()) {
        cout << "Failed to open file " << opts.calibrate_config << endl;
        return 4;
      }

      FileNode fCameraMatrix = fs["cameraMatrix"];
      if (fCameraMatrix.type() != FileNode::SEQ) {
        cout << "cameraMatrix is not a sequence!" << endl;
        return 4;
      }

      for (size_t index = 0; index < min(fCameraMatrix.size(), opts.file_paths.size()); ++index) {
        fCameraMatrix[index] >> cameraMatrix[index];
        WITH_DEBUG(
          cout << "Read matrix " << endl << cameraMatrix[index] << endl;
        )
      }

      FileNode fDistCoeffs = fs["distCoeffs"];
      if (fDistCoeffs.type() != FileNode::SEQ) {
        cout << "distCoeffs is not a sequence!" << endl;
        return 4;
      }
      for (size_t index = 0; index < min(fDistCoeffs.size(), opts.file_paths.size()); ++index) {
        fDistCoeffs[index] >> distCoeffs[index];
        WITH_DEBUG(
          cout << "Read matrix " << endl << distCoeffs[index] << endl;
        )
      }

      fs.release();
    }

    // Step #1. Remove distortion
    for (size_t i = 0; i < opts.file_paths.size(); ++i) {
      if (!distCoeffs[i].empty() && !cameraMatrix[i].empty()) {
        WITH_DEBUG(cout << "re-using existing undistort coeffs for camera #" << i << endl;)
        continue; // Use values from config file
      }

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
                chessboard_corners_orig_left[i])) {
              color = Scalar(0, 255, 255); // yellow
              chrono::duration<double, milli> elapsed = time - last_time;
              if (elapsed.count() >= opts.delay) {
                image_points.push_back(chessboard_corners_orig_left[i]);
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

    Mat status(Size(80 * opts.file_paths.size(), 40), projected[0].type());
    vector<Mat> frames(opts.file_paths.size());

    WITH_DEBUG(cout << "start searching for good frames" << endl;)

    // Step 2: Find good frames for alignment and stitching
    bool found_good_frames = false;
    while (!found_good_frames) {
      for (size_t i = 0; i < opts.file_paths.size(); ++i) {
        Mat t;
        videos[i] >> t;
        undistort(t, frames[i], cameraMatrix[i], distCoeffs[i]);
      }
      for (size_t i = 0; i < opts.file_paths.size(); ++i) {
        cv::Rect leftHalfRect(0, 0, frames[i].cols / 2, frames[i].rows);
        cv::Rect rightHalfRect(frames[i].cols / 2, 0, frames[i].cols / 2, frames[i].rows);
        Mat leftHalf = (StitchingMode::ChainOfTargets == opts.mode) ? frames[i](leftHalfRect) : frames[i];
        Mat rightHalf = frames[i](rightHalfRect);

        bool chessboardL = findChessboardCorners(leftHalf, chessboardSize,
            chessboard_corners_orig_left[i]);
        bool chessboardR = true;
        if (StitchingMode::ChainOfTargets == opts.mode)
          chessboardR = findChessboardCorners(rightHalf, chessboardSize,
              chessboard_corners_orig_right[i]);
        // red or orange
        Scalar colorL = chessboardL ? Scalar(0, 0, 255) : Scalar(0, 165, 255);
        Scalar colorR = chessboardR ? Scalar(0, 0, 255) : Scalar(0, 165, 255);

        if (chessboardL && chessboardR) {
          float angle = angleToHorizon(chessboard_corners_orig_left[i],
              chessboardSize);
          if (angle < (float)opts.angle) {
            colorL = colorR = Scalar(0, 255, 0); // green
          } else if (angle < 2.0 * opts.angle) {
            colorL = colorR = Scalar(0, 255, 255); // yellow
          }
          putText(frames[i], "Angle: " + std::to_string(angle), Point2f(10, 30),
              FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0 , 0));
        }
        rectangle(frames[i], Point2f(0, 0), Point2f(frames[i].cols / 2, frames[i].rows),
            colorL, 8);
        if (StitchingMode::ChainOfTargets == opts.mode) {
          rectangle(frames[i], Point2f(frames[i].cols / 2, 0), Point2f(frames[i].cols, frames[i].rows),
              colorR, 8);
          rectangle(status, Point2f(80 * i + 40, 0), Point2f(80 * (i + 1), 40), colorR, CV_FILLED);
        }
        // FIXME: status image is wrong
        rectangle(status, Point2f(80 * i, 0), Point2f(80 * i, 40), colorL, 20);
      }
      for (size_t i = 0; i < opts.file_paths.size(); ++i) {
        displayResult("Frame from camera " + std::to_string(i), frames[i]);
        waitKey(30);
      }
      imshow("Status", status);
      char r = waitKey(30);
      if (r == 'y') {
        for (size_t i = 0; i < opts.file_paths.size(); ++i) {
          images[i] = frames[i];
        }
        found_good_frames = true;
      }
    }
  }
  WITH_DEBUG(cout << "start calculating coeffs for stitching" << endl;)

  vector<char> isTransposed(opts.file_paths.size());
  for (size_t i = 0; i < opts.file_paths.size(); ++i) {
    Mat image = images[i];

    bool temp;
    if (!projectToTheFloor(image, chessboardSize,
        projected[i], chessboard_corners_orig_left[i],
        chessboard_corners_target_left[i], image_corners_target[i], temp)) {
      cout << "Failed to handle image #" << i + 1 << "!" << endl;
      return -1;
    }
    isTransposed[i] = temp;
    //chessboard_corners_target_left[i] = chessboard_corners_orig_left[i];

    WITH_DEBUG(
      cout << "Successfully projected " << i
          << "-th image to the floor" << endl;
    )
  }

  if (StitchingMode::ChainOfTargets == opts.mode) {
    for (size_t i = 0; i < opts.file_paths.size() - 1; ++i) {
      Rect rightHalfRect = Rect(images[i].cols / 2, 0,
              images[i].cols / 2, images[i].rows);
      Mat rightHalf = images[i](rightHalfRect);
      displayResult("right half", rightHalf, true);

      vector<Point2f> chessboard_points;
      if (!findChessboardCorners(rightHalf, chessboardSize, chessboard_points)) {
        cout << "Failed to detect right chessboard on image #"
            << i + 1 << "!" << endl;
        return 2;
      }
      WITH_DEBUG(
        cout << "Right chessboard: " <<  endl;
        for (auto &row : chessboard_points) {
          cout << "\t: " << row << endl;
        }
      )
      chessboard_corners_orig_right[i] = extractCorners(
          orderChessboardCorners(chessboard_points, chessboardSize));
      for (auto &P : chessboard_corners_orig_right[i]) {
        P.x += images[i].cols / 2;
      }

#if 1
  {
    bool K = isTransposed[i];
    Mat temp;
    images[i].copyTo(temp);
    auto ordered = orderChessboardCorners(chessboard_points, chessboardSize);
    int r = 1;
    for (size_t i = 0; i < ordered.size(); ++i) {
      for (size_t j = 0; j < ordered[i].size(); ++j) {
        cv::Point2f &P = ordered[i][j];
        P.x += temp.cols / 2;
        cv::circle(temp, P, (r * 5), cv::Scalar(200, 250, 250), 3);
        ++r;
      }
    }
    displayResult("right", temp, true);
  }
#endif
    }
  }

  // We have two chessboards per image: left and right
  // Left is used to project image to the floor pane
  // Right is used to connect image with the next one: left board on i-th
  // image should be placed at right board on (i-1)-th image

  if (StitchingMode::ChainOfTargets == opts.mode) {
    Rect rightHalfRect = Rect(projected[0].cols / 2, 0,
            projected[0].cols / 2, projected[0].rows);
    Mat rightHalf = projected[0](rightHalfRect);

    vector<Point2f> chessboard_points;
    if (!findChessboardCorners(rightHalf, chessboardSize, chessboard_points)) {
      cout << "Failed to detect right chessboard on image #"
          << 1 << "!" << endl;
      return 2;
    }
    chessboard_corners_target_right[0] = extractCorners(
        orderChessboardCorners(chessboard_points, chessboardSize));
    for (auto &P : chessboard_corners_target_right[0]) {
      P.x += projected[0].cols / 2;
    }
  }
  Size result_size(projected.front().cols, projected.front().rows);

  for (size_t i = 1; i < opts.file_paths.size(); ++i) {
    auto target_chessboard_corners =
      (StitchingMode::ChainOfTargets == opts.mode)
          ? chessboard_corners_target_right[i - 1]
          : chessboard_corners_target_left[0];

    // Left chessboard of i-th image should be placed at right chessboard
    // on (i-1)-th image
    WITH_DEBUG(
      cout << "Initial estimate: Trying to find homography between " << endl
        << chessboard_corners_target_left[i] << " and " << endl
        << target_chessboard_corners << endl;
    )
    Mat preH = findHomography(chessboard_corners_target_left[i],
        target_chessboard_corners, CV_RANSAC);

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
    if (minX < 0) {
      dx = fabs(minX);
    }
    if (minY < 0) {
      dy = fabs(minY);
    }

    for (size_t h = 0; h < chessboard_corners_target_left[0].size(); ++h) {
      chessboard_corners_target_left[0][h].x += dx;
      chessboard_corners_target_left[0][h].y += dy;
    }
    if (StitchingMode::ChainOfTargets == opts.mode) {
      for (int j = 0; j < i; ++j) {
        for (size_t h = 0; h < chessboard_corners_target_right[j].size(); ++h) {
          chessboard_corners_target_right[j][h].x += dx;
          chessboard_corners_target_right[j][h].y += dy;
        }
      }
    }

    H[0] = findHomography(chessboard_corners_orig_left[0],
        chessboard_corners_target_left[0], CV_RANSAC);
    WITH_DEBUG(
      cout << "Adjust: Trying to find homography between " << endl
        << chessboard_corners_orig_left[0] << " and " << endl
        << chessboard_corners_target_left[0] << endl;
    )
    for (int j = 1; j <= i; ++j) {
      auto local_target_chessboard_corners =
          (StitchingMode::ChainOfTargets == opts.mode)
              ? chessboard_corners_target_right[j - 1]
              : chessboard_corners_target_left[0];

      WITH_DEBUG(
        cout << "Adjust: Trying to find homography between " << endl
          << chessboard_corners_orig_left[j] << " and " << endl
          << local_target_chessboard_corners << endl;
      )
      H[j] = findHomography(chessboard_corners_orig_left[j],
          local_target_chessboard_corners, CV_RANSAC);
    }


    cout << "result_size before: " << result_size << std::endl;
    cout << "maxX, dx, maxY, dy " << maxX << " " << dx << " " << maxY << " " << dy << std::endl;
    result_size = Size(max((float)result_size.width, maxX),
      max((float)result_size.height, maxY));
    result_size.width += dx;
    result_size.height += dy;
    cout << "result_size: " << result_size << std::endl;

    Mat intermediate(result_size, projected[0].type());
    for (int j = 0; j <= i; ++j) {
      warpPerspective(images[j], intermediate, H[j], result_size, INTER_LINEAR,
          BORDER_TRANSPARENT);
    }
    if (StitchingMode::ChainOfTargets == opts.mode) {
      // chessboard_corners_target_right[i]
      //     = warp(chessboard_corners_orig_right[i], H[i])
      Mat t;
      perspectiveTransform(Mat(chessboard_corners_orig_right[i]), t, H[i]);
      chessboard_corners_target_right[i] = (vector<Point2f>)t;
    }
    displayResult("intermediate", intermediate, true);
  }

  FileStorage fs(opts.stitch_config, FileStorage::WRITE);

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
