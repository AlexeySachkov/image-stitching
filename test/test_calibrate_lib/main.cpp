#include "utils.hpp"

#include "gtest/gtest.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>
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

enum class Direction {
  INC, DEC
};

void validate(const std::vector<std::vector<cv::Point2f>> &data,
    const cv::Size &size) {
  ASSERT_EQ(data.size(), size.height);
  for (size_t i = 0; i < data.size(); ++i) {
    ASSERT_EQ(data[i].size(), size.width);
  }
  for (int i = 0; i < size.height; ++i) {
    for (int j = 0; j < size.width; ++j) {
      int nx = j - 1;
      if (nx >= 0) {
        ASSERT_GT(data[i][j].x, data[i][nx].x)
            << "i,j = (" << i << ", " << j << ");"
            << "ny, nx = (" << i << ", " << nx << ")";
      }
      int ny = i - 1;
      if (ny >= 0) {
        ASSERT_GT(data[i][j].y, data[ny][j].y)
            << "i,j = (" << i << ", " << j << ");"
            << "ny, nx = (" << ny << ", " << j << ")";
      }
    }
  }
}

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

std::vector<std::vector<cv::Point2f>> transposedBoard(const cv::Size &size) {
  std::vector<std::vector<cv::Point2f>> result(size.height);
  for (int i = 0; i < size.height; ++i) {
    result[i] = column(size.height - i, size.width, Direction::INC);
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
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
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
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
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
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
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
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
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
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
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
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
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
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
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
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Transposed_Test1) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by rows
  // rows ordered from top to bottom
  // columns ordered from left to right
  append(input, row(1, boardSize.height, Direction::INC));
  append(input, row(2, boardSize.height, Direction::INC));
  append(input, row(3, boardSize.height, Direction::INC));
  append(input, row(4, boardSize.height, Direction::INC));
  append(input, row(5, boardSize.height, Direction::INC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = transposedBoard(boardSize);
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Transposed_Test2) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by rows
  // rows ordered from top to bottom
  // columns ordered from right to left
  append(input, row(1, boardSize.height, Direction::DEC));
  append(input, row(2, boardSize.height, Direction::DEC));
  append(input, row(3, boardSize.height, Direction::DEC));
  append(input, row(4, boardSize.height, Direction::DEC));
  append(input, row(5, boardSize.height, Direction::DEC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = transposedBoard(boardSize);
  ASSERT_EQ(result, expected) << "input: " << input << "\n"
      << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Transposed_Test3) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by rows
  // rows ordered from bottom to top
  // columns ordered from left to right
  append(input, row(5, boardSize.height, Direction::INC));
  append(input, row(4, boardSize.height, Direction::INC));
  append(input, row(3, boardSize.height, Direction::INC));
  append(input, row(2, boardSize.height, Direction::INC));
  append(input, row(1, boardSize.height, Direction::INC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = transposedBoard(boardSize);
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;

}

TEST(OrderChessboardCorners, Transposed_Test4) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by rows
  // rows ordered from bottom to top
  // columns ordered from right to left
  append(input, row(5, boardSize.height, Direction::DEC));
  append(input, row(4, boardSize.height, Direction::DEC));
  append(input, row(3, boardSize.height, Direction::DEC));
  append(input, row(2, boardSize.height, Direction::DEC));
  append(input, row(1, boardSize.height, Direction::DEC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = transposedBoard(boardSize);
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Transposed_Test5) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by columns
  // rows ordered from top to bottom
  // columns ordered from left to right
  append(input, column(1, boardSize.width, Direction::INC));
  append(input, column(2, boardSize.width, Direction::INC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = transposedBoard(boardSize);
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Transposed_Test6) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by columns
  // rows ordered from top to bottom
  // columns ordered from right to left
  append(input, column(2, boardSize.width, Direction::INC));
  append(input, column(1, boardSize.width, Direction::INC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = transposedBoard(boardSize);
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Transposed_Test7) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by columns
  // rows ordered from bottom to top
  // columns ordered from left to right
  append(input, column(1, boardSize.width, Direction::DEC));
  append(input, column(2, boardSize.width, Direction::DEC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = transposedBoard(boardSize);
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
}

TEST(OrderChessboardCorners, Transposed_Test8) {
  cv::Size boardSize(5, 2); // two rows, five columns
  std::vector<cv::Point2f> input;
  // by columns
  // rows ordered from bottom to top
  // columns ordered from right to left
  append(input, column(2, boardSize.width, Direction::DEC));
  append(input, column(1, boardSize.width, Direction::DEC));

  auto result = orderChessboardCorners(input, boardSize);
  auto expected = transposedBoard(boardSize);
  ASSERT_EQ(result, expected)
      << "result: " << result << "expected: " << expected;
}

#define _STRINGIFY(X) #X
#define STRINGIFY(X) _STRINGIFY(X)

TEST(OrderChessboardCorners, RealImage1) {
  cv::Mat image = cv::imread(
      std::string(STRINGIFY(INPUTS_DIR)) + "/chessboard_corners/3x4.jpg");
  ASSERT_FALSE(image.empty());
  std::vector<cv::Point2f> chessboardCorners;
  cv::Size boardSize(3, 4);
  ASSERT_TRUE(findChessboardCorners(image, boardSize, chessboardCorners));
  auto result = orderChessboardCorners(chessboardCorners, boardSize);
  validate(result, boardSize);
  if (HasFatalFailure())
    FAIL() << "result: " << result;
}

TEST(OrderChessboardCorners, RealImage2) {
  cv::Mat image = cv::imread(
      std::string(STRINGIFY(INPUTS_DIR)) + "/chessboard_corners/3x5.jpg");
  ASSERT_FALSE(image.empty());
  std::vector<cv::Point2f> chessboardCorners;
  cv::Size boardSize(3, 5);
  ASSERT_TRUE(findChessboardCorners(image, boardSize, chessboardCorners));
  auto result = orderChessboardCorners(chessboardCorners, boardSize);
  validate(result, boardSize);
  if (HasFatalFailure())
    FAIL() << "result: " << result;
}

TEST(OrderChessboardCorners, RealImage3) {
  cv::Mat image = cv::imread(
      std::string(STRINGIFY(INPUTS_DIR)) + "/chessboard_corners/5x3.jpg");
  ASSERT_FALSE(image.empty());
  std::vector<cv::Point2f> chessboardCorners;
  cv::Size boardSize(5, 3);
  ASSERT_TRUE(findChessboardCorners(image, boardSize, chessboardCorners));
  auto result = orderChessboardCorners(chessboardCorners, boardSize);
  validate(result, boardSize);
  if (HasFatalFailure())
    FAIL() << "result: " << result;
}
