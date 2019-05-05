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
    cout << "Usage: " << argv[0] << " -c=/path/to/conf.xml /path/to/img1.jpg /path/to/img2.jpg";
    return 1;
  }

  FileStorage fs(opts.output_file, FileStorage::READ);

  if (!fs.isOpened()) {
    cout << "Failed to open configuration file " << opts.output_file << endl;
    return 2;
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

  fs.release();

  Mat img0 = imread(opts.file_paths.front());
  Mat result(result_size, img0.type());

  for (size_t i = 0; i < opts.file_paths.size(); ++i) {
    Mat image = imread(opts.file_paths[i]);
    warpPerspective(image, result, H[i], result_size, INTER_LINEAR, BORDER_TRANSPARENT);
    displayResult("temp", result, true);
  }

  displayResult("Final", result, true);

  return 0;
}
