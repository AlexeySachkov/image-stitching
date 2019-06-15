#include "utils.hpp"

#include "gtest/gtest.h"

#include <iostream>
#include <vector>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
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

std::ostream& operator<<(std::ostream& os,
    const std::vector<std::vector<cv::Point2f>> &v) {
  os << "[\n";
  for (size_t i = 0; i < v.size(); ++i) {
    for (size_t j = 0; j < v[i].size(); ++j) {
      os << v[i][j] << "; ";
    }
    os << "\n";
  }
  os << "]\n";
  return os;
}

static void validate(const std::vector<std::vector<cv::Point2f>> &result,
    const std::vector<std::vector<cv::Point2f>> &expected) {
  ASSERT_EQ(result.size(), expected.size());
  for (size_t i = 0; i < result.size(); ++i) {
    ASSERT_EQ(result[i].size(), expected[i].size()) << "i = " << i;
  }

  for (size_t i = 0; i < result.size(); ++i) {
    for (size_t j = 0; j < result[i].size(); ++j) {
      ASSERT_EQ(result[i][j], expected[i][j]) << "i = " << i << ", j = " << j;
    }
  }
}

TEST(OrderChessboardCorners, Test1) {
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

  auto result = orderChessboardCorners(input, boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test2) {
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

  auto result = orderChessboardCorners(input, boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test3) {
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

  auto result = orderChessboardCorners(input, boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test4) {
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

  auto result = orderChessboardCorners(input, boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test5) {
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

  auto result = orderChessboardCorners(input, boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test6) {
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

  auto result = orderChessboardCorners(input, boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test7) {
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

  auto result = orderChessboardCorners(input, boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test8) {
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

  auto result = orderChessboardCorners(input, boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test9) {
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

  auto result = orderChessboardCorners(input, boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}
