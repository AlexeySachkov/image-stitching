
#ifndef __INCLUDE_OPTS_HPP__
#define __INCLUDE_OPTS_HPP__

#include <string>
#include <vector>

enum class StitchingMode {
  OneCommonTarget,
  ChainOfTargets
};

struct command_line_opts {
  unsigned verbosity = 0;
  bool interactive = false;
  std::vector<std::string> file_paths;
  bool video = false;

  int delay = 300;
  int number_of_frames = 30;

  int board_width = 5;
  int board_height = 3;

  int angle = 1;

  StitchingMode mode = StitchingMode::ChainOfTargets;

  std::string calibrate_config;
  std::string stitch_config = "stitch.conf.xml";
};

bool parse_command_line_opts(int argc, char *argv[]);

#endif // __INCLUDE_OPTS_HPP__
