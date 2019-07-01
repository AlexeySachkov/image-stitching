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
    cout << "Usage: " << argv[0] << " -c=/path/to/conf.xml";
    return 1;
  }

  FileStorage fs(opts.stitch_config, FileStorage::READ);

  if (!fs.isOpened()) {
    cout << "Failed to open configuration file " << opts.stitch_config << endl;
    return 2;
  }

  fs["video"] >> opts.video;
  FileNode fNames = fs["file_paths"];
  if (fNames.type() != FileNode::SEQ) {
    cout << "file_paths is not a sequence!" << endl;
    return 4;
  }

  opts.file_paths.resize(fNames.size());
  for (size_t index = 0; index < fNames.size(); ++index) {
    fNames[index] >> opts.file_paths[index];
    WITH_DEBUG(
      cout << "Read file path " << endl << opts.file_paths[index] << endl;
    )
  }

  if (opts.file_paths.empty()) {
    cout << "Usage: " << argv[0] << " /path/to/img1.jpg /path/to/img2.jpg";
    return 3;
  }

  Size result_size;

  fs["result_size"] >> result_size;
  WITH_DEBUG(
    cout << "Result size: " << result_size << endl;
  )

  vector<Mat> H(opts.file_paths.size());
  FileNode fH = fs["H"];
  if (fH.type() != FileNode::SEQ)
  {
    cout << "H is not a sequence!" << endl;
    return 4;
  }

  for (size_t index = 0; index < fH.size(); ++index) {
    fH[index] >> H[index];
    WITH_DEBUG(
      cout << "Read matrix " << endl << H[index] << endl;
    )
  }

  vector<Mat> cameraMatrix(opts.file_paths.size());

  FileNode fCameraMatrix = fs["cameraMatrix"];
  if (fCameraMatrix.type() != FileNode::SEQ) {
    cout << "cameraMatrix is not a sequence!" << endl;
    return 4;
  }

  for (size_t index = 0; index < fCameraMatrix.size(); ++index) {
    fCameraMatrix[index] >> cameraMatrix[index];
    WITH_DEBUG(
      cout << "Read matrix " << endl << cameraMatrix[index] << endl;
    )
  }

  vector<Mat> distCoeffs(opts.file_paths.size());
  FileNode fDistCoeffs = fs["distCoeffs"];
  if (fDistCoeffs.type() != FileNode::SEQ) {
    cout << "distCoeffs is not a sequence!" << endl;
    return 4;
  }

  for (size_t index = 0; index < fDistCoeffs.size(); ++index) {
    fDistCoeffs[index] >> distCoeffs[index];
    WITH_DEBUG(
      cout << "Read matrix " << endl << distCoeffs[index] << endl;
    )
  }

  fs.release();

  if (!opts.video) {
    Mat img0 = imread(opts.file_paths.front());
    Mat result(result_size, img0.type());

    auto start = chrono::steady_clock::now();
    auto end = chrono::steady_clock::now();

    for (size_t i = 0; i < opts.file_paths.size(); ++i) {
      Mat image = imread(opts.file_paths[i]);
      start = chrono::steady_clock::now();
      warpPerspective(image, result, H[i], result_size, INTER_LINEAR,
          BORDER_TRANSPARENT);
      end = chrono::steady_clock::now();
      cout << chrono::duration<double, milli>(end - start).count() << endl;
      displayResult("temp", result, true);
    }


    displayResult("Final", result, true);
    imwrite("final.jpg", result);
  } else {
    vector<VideoCapture> videos(opts.file_paths.size());
    for (size_t i = 0; i < opts.file_paths.size(); ++i) {
      videos[i].open(opts.file_paths[i]);
      if (!videos[i].isOpened()) {
        cout << "Failed to open file " << opts.file_paths[i] << "!" << endl;
        return 5;
      }
    }

    Mat t;
    videos.front() >> t;
    int type = t.type();
    for (size_t i = 1; i < opts.file_paths.size(); ++i) {
      videos[i] >> t; // skip first frame
    }

    while (true) {
      Mat result(result_size, type);
      for (size_t i = 0; i < videos.size(); ++i) {
        Mat frame;
        videos[i] >> frame;
        Mat undistorted;
        undistort(frame, undistorted, cameraMatrix[i], distCoeffs[i]);
        warpPerspective(undistorted, result, H[i], result_size, INTER_LINEAR,
            BORDER_TRANSPARENT);
      }
      displayResult("Final", result);
      waitKey(30);
    }
  }

  return 0;
}
