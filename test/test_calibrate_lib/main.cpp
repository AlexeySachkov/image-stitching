#include "utils.hpp"

#include <iostream>
#include <vector>
#include <string>

typedef bool (*test_fn)(std::string &);

static bool test_1(std::string &log);
static bool test_2(std::string &log);
static bool test_3(std::string &log);
static bool test_4(std::string &log);
static bool test_5(std::string &log);
static bool test_6(std::string &log);
static bool test_7(std::string &log);
static bool test_8(std::string &log);

int main() {
  std::vector<test_fn> test_functions;
  std::string test_log;

  test_functions.push_back(&test_1);
  test_functions.push_back(&test_2);
  test_functions.push_back(&test_3);
  test_functions.push_back(&test_4);
  test_functions.push_back(&test_5);
  test_functions.push_back(&test_6);
  test_functions.push_back(&test_7);
  test_functions.push_back(&test_8);

  size_t failed = 0;

  for (size_t i = 0; i < test_functions.size(); ++i) {
    test_log = "";
    if (!test_functions[i](test_log)) {
      std::cout << "Test " << i << " failed. Logs: " << std::endl;
      std::cout << test_log << std::endl;
      ++failed;
    } else {
      std::cout << "Test " << i << " passed." << std::endl;
    }
  }

  if (failed != 0) {
    std::cout << failed << " tests failed out of " << test_functions.size()
        << std::endl;
    return 1;
  } else {
    std::cout << "All " << test_functions.size() << " tests passed"
        << std::endl;
  }

  return 0;
}

static void print(const std::vector<std::vector<cv::Point2f>> &data,
    std::string &log) {
  log += "{\n";
  for (size_t i = 0; i < data.size(); ++i) {
    log += "\t";
    for (size_t j = 0; j < data[i].size(); ++j) {
        log += " (" + std::to_string(data[i][j].x) + ", "
            + std::to_string(data[i][j].y) + ")";
    }
    log += "\n";
  }
  log += "}\n";
}

static bool validate(const std::vector<std::vector<cv::Point2f>> &result,
    const std::vector<std::vector<cv::Point2f>> &expected, std::string &log) {
  if (result.size() != expected.size()) {
    log += "result.size() != expected.size():\n\t"
        + std::to_string(result.size()) + " != "
        + std::to_string(expected.size()) + "\n";
    print(result, log);
    print(expected, log);
    return false;
  }
  for (size_t i = 0; i < result.size(); ++i) {
    if (result[i].size() != expected[i].size()) {
      log += "result[i].size() != expected[i].size(): [i == "
          + std::to_string(i) + "\n\t"
          + std::to_string(result[i].size()) + " != "
          + std::to_string(expected[i].size()) + "\n";
      print(result, log);
      print(expected, log);
      return false;
    }
  }
  for (size_t i = 0; i < result.size(); ++i) {
    for (size_t j = 0; j < result[i].size(); ++j) {
      if (result[i][j] != expected[i][j]) {
        log += "result[i][j] != expected[i][j]: [i == "
            + std::to_string(i) + ", j == " + std::to_string(j)
            + "\n\t" + std::to_string(result[i][j].x) + ", "
            + std::to_string(result[i][j].y)
            + " != " + std::to_string(expected[i][j].x) + ", "
            + std::to_string(expected[i][j].y) + "\n";
        print(result, log);
        print(expected, log);
        return false;
      }
    }
  }

  return true;
}

static bool test_1(std::string &log) {
  cv::Size boardSize(3, 2); // two rows, three columns
  std::vector<cv::Point2f> input = {
    // by rows
    // rows ordered from top to bottom
    // columns ordered from left to right
    cv::Point2f(1, 1), cv::Point2f(2, 1), cv::Point2f(3, 1),
    cv::Point2f(1, 2), cv::Point2f(2, 2), cv::Point2f(3, 2)
  };

  std::vector<std::vector<cv::Point2f>> expected = {
    // by rows
    // rows ordered from top to bottom
    // columns ordered from left to right
    { cv::Point2f(1, 1), cv::Point2f(2, 1), cv::Point2f(3, 1) },
    { cv::Point2f(1, 2), cv::Point2f(2, 2), cv::Point2f(3, 2) }
  };

  return validate(orderChessboardCorners(input, boardSize), expected, log);
}

static bool test_2(std::string &log) {
  cv::Size boardSize(3, 2); // two rows, three columns
  std::vector<cv::Point2f> input = {
    // by rows
    // rows ordered from top to bottom
    // columns ordered from right to left
    cv::Point2f(3, 1), cv::Point2f(2, 1), cv::Point2f(1, 1),
    cv::Point2f(3, 2), cv::Point2f(2, 2), cv::Point2f(1, 2)
  };

  std::vector<std::vector<cv::Point2f>> expected = {
    // by rows
    // rows ordered from top to bottom
    // columns ordered from left to right
    { cv::Point2f(1, 1), cv::Point2f(2, 1), cv::Point2f(3, 1) },
    { cv::Point2f(1, 2), cv::Point2f(2, 2), cv::Point2f(3, 2) }
  };

  return validate(orderChessboardCorners(input, boardSize), expected, log);
}

static bool test_3(std::string &log) {
  cv::Size boardSize(3, 2); // two rows, three columns
  std::vector<cv::Point2f> input = {
    // by rows
    // rows ordered from bottom to top
    // columns ordered from left to right
    cv::Point2f(1, 2), cv::Point2f(2, 2), cv::Point2f(3, 2),
    cv::Point2f(1, 1), cv::Point2f(2, 1), cv::Point2f(3, 1)
  };

  std::vector<std::vector<cv::Point2f>> expected = {
    // by rows
    // rows ordered from top to bottom
    // columns ordered from left to right
    { cv::Point2f(1, 1), cv::Point2f(2, 1), cv::Point2f(3, 1) },
    { cv::Point2f(1, 2), cv::Point2f(2, 2), cv::Point2f(3, 2) }
  };

  return validate(orderChessboardCorners(input, boardSize), expected, log);
}

static bool test_4(std::string &log) {
  cv::Size boardSize(3, 2); // two rows, three columns
  std::vector<cv::Point2f> input = {
    // by rows
    // rows ordered from bottom to top
    // columns ordered from right to left
    cv::Point2f(3, 2), cv::Point2f(2, 2), cv::Point2f(1, 2),
    cv::Point2f(3, 1), cv::Point2f(2, 1), cv::Point2f(1, 1)
  };

  std::vector<std::vector<cv::Point2f>> expected = {
    // by rows
    // rows ordered from top to bottom
    // columns ordered from left to right
    { cv::Point2f(1, 1), cv::Point2f(2, 1), cv::Point2f(3, 1) },
    { cv::Point2f(1, 2), cv::Point2f(2, 2), cv::Point2f(3, 2) }
  };

  return validate(orderChessboardCorners(input, boardSize), expected, log);
}

static bool test_5(std::string &log) {
  cv::Size boardSize(3, 2); // two rows, three columns
  std::vector<cv::Point2f> input = {
    // by columns
    // rows ordered from top to bottom
    // columns ordered from left to right
    cv::Point2f(1, 1), cv::Point2f(1, 2),
    cv::Point2f(2, 1), cv::Point2f(2, 2),
    cv::Point2f(3, 1), cv::Point2f(3, 2)
  };

  std::vector<std::vector<cv::Point2f>> expected = {
    // by rows
    // rows ordered from top to bottom
    // columns ordered from left to right
    { cv::Point2f(1, 1), cv::Point2f(2, 1), cv::Point2f(3, 1) },
    { cv::Point2f(1, 2), cv::Point2f(2, 2), cv::Point2f(3, 2) }
  };

  return validate(orderChessboardCorners(input, boardSize), expected, log);
}

static bool test_6(std::string &log) {
  cv::Size boardSize(3, 2); // two rows, three columns
  std::vector<cv::Point2f> input = {
    // by columns
    // rows ordered from bottom to top
    // columns ordered from left to right
    cv::Point2f(1, 2), cv::Point2f(1, 1),
    cv::Point2f(2, 2), cv::Point2f(2, 1),
    cv::Point2f(3, 2), cv::Point2f(3, 1)
  };

  std::vector<std::vector<cv::Point2f>> expected = {
    // by rows
    // rows ordered from top to bottom
    // columns ordered from left to right
    { cv::Point2f(1, 1), cv::Point2f(2, 1), cv::Point2f(3, 1) },
    { cv::Point2f(1, 2), cv::Point2f(2, 2), cv::Point2f(3, 2) }
  };

  return validate(orderChessboardCorners(input, boardSize), expected, log);
}

static bool test_7(std::string &log) {
  cv::Size boardSize(3, 2); // two rows, three columns
  std::vector<cv::Point2f> input = {
    // by columns
    // rows ordered from top to bottom
    // columns ordered from right to left
    cv::Point2f(3, 1), cv::Point2f(3, 2),
    cv::Point2f(2, 1), cv::Point2f(2, 2),
    cv::Point2f(1, 1), cv::Point2f(1, 2)
  };

  std::vector<std::vector<cv::Point2f>> expected = {
    // by rows
    // rows ordered from top to bottom
    // columns ordered from left to right
    { cv::Point2f(1, 1), cv::Point2f(2, 1), cv::Point2f(3, 1) },
    { cv::Point2f(1, 2), cv::Point2f(2, 2), cv::Point2f(3, 2) }
  };

  return validate(orderChessboardCorners(input, boardSize), expected, log);
}

static bool test_8(std::string &log) {
  cv::Size boardSize(3, 2); // two rows, three columns
  std::vector<cv::Point2f> input = {
    // by columns
    // rows ordered from bottom to top
    // columns ordered from right to left
    cv::Point2f(3, 2), cv::Point2f(3, 1),
    cv::Point2f(2, 2), cv::Point2f(2, 1),
    cv::Point2f(1, 2), cv::Point2f(1, 1)
  };

  std::vector<std::vector<cv::Point2f>> expected = {
    // by rows
    // rows ordered from top to bottom
    // columns ordered from left to right
    { cv::Point2f(1, 1), cv::Point2f(2, 1), cv::Point2f(3, 1) },
    { cv::Point2f(1, 2), cv::Point2f(2, 2), cv::Point2f(3, 2) }
  };

  return validate(orderChessboardCorners(input, boardSize), expected, log);
}
