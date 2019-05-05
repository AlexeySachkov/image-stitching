cmake_minimum_required(VERSION 3.11)

project("ImageStitching" CXX)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

message(STATUS "Found OpenCV ${OpenCV_VERSION}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
message(STATUS "OpenCV_INLCUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

# Global include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include/)

add_subdirectory(lib)
add_subdirectory(tools)

enable_testing()
add_subdirectory(test)

