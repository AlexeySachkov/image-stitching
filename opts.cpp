
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
      std::string verbosity = arg.substr(pos);
      opts.verbosity = atoi(verbosity.c_str());
    } else {
      // assume argument is a path to an image
      opts.file_paths.push_back(arg);
    }
  }

  return valid;
}
