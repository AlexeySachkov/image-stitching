
#include "opts.hpp"

command_line_opts opts;

bool parse_command_line_opts(int argc, char *argv[]) {
  bool valid = true;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if ("--interactive" == arg || "-i" == arg) {
      opts.interactive = true;
    } else if (arg.find("--verbosity") == 0 || arg.find("-v") == 0) {
      std::string::size_type pos = arg.find("=");
      if (std::string::npos == pos) {
        valid = false;
        break;
      }

      std::string verbosity = arg.substr(pos + 1);
      opts.verbosity = atoi(verbosity.c_str());
    } else if (arg.find("--board") == 0 || arg.find("-b") == 0) {
      std::string::size_type pos = arg.find("=");
      if (std::string::npos == pos) {
        valid = false;
        break;
      }

      std::string board_size = arg.substr(pos + 1);
      pos = board_size.find("x");
      if (std::string::npos == pos) {
        valid = false;
        break;
      }

      std::string board_width = board_size.substr(0, pos);
      std::string board_height = board_size.substr(pos + 1);

      opts.board_width = atoi(board_width.c_str());
      opts.board_height = atoi(board_height.c_str());
    } else if (arg.find("--output") == 0 || arg.find("-o") == 0) {
      std::string::size_type pos = arg.find("=");
      if (std::string::npos == pos) {
        valid = false;
        break;
      }

      opts.output_file = arg.substr(pos + 1);
    } else if ("--video" == arg) {
      opts.video = true;
    } else if (arg.find("--delay") == 0) {
      std::string::size_type pos = arg.find("=");
      if (std::string::npos == pos) {
        valid = false;
        break;
      }

      opts.delay = atoi(arg.substr(pos + 1).c_str());
    } else if (arg.find("--num") == 0) {
      std::string::size_type pos = arg.find("=");
      if (std::string::npos == pos) {
        valid = false;
        break;
      }

      opts.number_of_frames = atoi(arg.substr(pos + 1).c_str());
    } else {
      // assume argument is a path to an image
      opts.file_paths.push_back(arg);
    }
  }

  return valid;
}
