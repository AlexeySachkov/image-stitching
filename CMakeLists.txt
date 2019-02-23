cmake_minimum_required(VERSION 3.11)

project("ImageStitching" CXX)

find_package(OpenCV REQUIRED)

set(SOURCE_FILES
  main.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(stitch ${SOURCE_FILES})

target_link_libraries(stitch ${OpenCV_LIBS})

install(TARGETS stitch
  RUNTIME DESTINATION bin)
