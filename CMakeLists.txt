cmake_minimum_required(VERSION 3.11)

project("ImageStitching" CXX)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

message(STATUS "Found OpenCV ${OpenCV_VERSION}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
message(STATUS "OpenCV_INLCUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

set(SOURCE_FILES
  main.cpp
  utils.cpp)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include/)

add_executable(stitch ${SOURCE_FILES})

target_link_libraries(stitch ${OpenCV_LIBS})

install(TARGETS stitch
  RUNTIME DESTINATION bin)
