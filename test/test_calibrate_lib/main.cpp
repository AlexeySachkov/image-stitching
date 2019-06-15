#include "utils.hpp"

#include "gtest/gtest.h"

#include <iostream>
#include <vector>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// TODO: tests for areThereNMonotonousPoints
// TODO: tests for getPointsOrientation

std::ostream& operator<<(std::ostream& os,
    const std::vector<std::vector<cv::Point2f>> &v) {
  os << "\n[\n";
  for (size_t i = 0; i < v.size(); ++i) {
    os << "\t";
    for (size_t j = 0; j < v[i].size(); ++j) {
      os << v[i][j] << "; ";
    }
    os << "\n";
  }
  os << "]\n";
  return os;
}

namespace {

void validate(const std::vector<std::vector<cv::Point2f>> &result,
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

enum class Direction {
  INC, DEC
};

std::vector<cv::Point2f> row(float Y, int N, Direction d) {
  std::vector<cv::Point2f> result(N);
  float step = (d == Direction::INC) ? 1.0 : -1.0;
  float X = (d == Direction::INC) ? 1.0 : (float)N;
  for (int i = 0; i < N; ++i, X += step) {
    result[i] = cv::Point2f(X, Y);
  }
  return result;
}

std::vector<cv::Point2f> column(float X, int N, Direction d) {
  std::vector<cv::Point2f> result(N);
  float step = (d == Direction::INC) ? 1.0 : -1.0;
  float Y = (d == Direction::INC) ? 1.0 : (float)N;
  for (int i = 0; i < N; ++i, Y += step) {
    result[i] = cv::Point2f(X, Y);
  }
  return result;
}

std::vector<std::vector<cv::Point2f>> board(const cv::Size &size) {
  std::vector<std::vector<cv::Point2f>> result(size.height);
  for (int i = 0; i < size.height; ++i) {
    result[i] = row(i + 1, size.width, Direction::INC);
  }

  return result;
}

void append(std::vector<cv::Point2f> &v, const std::vector<cv::Point2f> &n) {
  for (const auto &item : n) {
    v.push_back(item);
  }
}

}

TEST(OrderChessboardCorners, Test1) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by rows
  // rows ordered from top to bottom
  // columns ordered from left to right
  append(input, row(1, boardSize.width, Direction::INC));
  append(input, row(2, boardSize.width, Direction::INC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = board(boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test2) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by rows
  // rows ordered from bottom to top
  // columns ordered from left to right
  append(input, row(2, boardSize.width, Direction::INC));
  append(input, row(1, boardSize.width, Direction::INC));


  auto result = orderChessboardCorners(input, boardSize);
  auto expected = board(boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test3) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by rows
  // rows ordered from top to bottom
  // columns ordered from right to left
  append(input, row(1, boardSize.width, Direction::DEC));
  append(input, row(2, boardSize.width, Direction::DEC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = board(boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test4) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
    // by rows
    // rows ordered from bottom to top
    // columns ordered from right to left
  append(input, row(2, boardSize.width, Direction::DEC));
  append(input, row(1, boardSize.width, Direction::DEC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = board(boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test5) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by columns
  // rows ordered from top to bottom
  // columns ordered from left to right
  append(input, column(1, boardSize.height, Direction::INC));
  append(input, column(2, boardSize.height, Direction::INC));
  append(input, column(3, boardSize.height, Direction::INC));
  append(input, column(4, boardSize.height, Direction::INC));
  append(input, column(5, boardSize.height, Direction::INC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = board(boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test6) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by columns
  // rows ordered from bottom to top
  // columns ordered from left to right
  append(input, column(1, boardSize.height, Direction::DEC));
  append(input, column(2, boardSize.height, Direction::DEC));
  append(input, column(3, boardSize.height, Direction::DEC));
  append(input, column(4, boardSize.height, Direction::DEC));
  append(input, column(5, boardSize.height, Direction::DEC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = board(boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test7) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by columns
  // rows ordered from top to bottom
  // columns ordered from right to left
  append(input, column(5, boardSize.height, Direction::INC));
  append(input, column(4, boardSize.height, Direction::INC));
  append(input, column(3, boardSize.height, Direction::INC));
  append(input, column(2, boardSize.height, Direction::INC));
  append(input, column(1, boardSize.height, Direction::INC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = board(boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Test8) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by columns
  // rows ordered from bottom to top
  // columns ordered from right to left
  append(input, column(5, boardSize.height, Direction::DEC));
  append(input, column(4, boardSize.height, Direction::DEC));
  append(input, column(3, boardSize.height, Direction::DEC));
  append(input, column(2, boardSize.height, Direction::DEC));
  append(input, column(1, boardSize.height, Direction::DEC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = board(boardSize);
  validate(result, expected);
  if (HasFatalFailure())
    FAIL() << "result: " << result << "expected: " << expected;
}
