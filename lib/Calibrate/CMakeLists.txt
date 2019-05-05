
set(TARGET_NAME Calibrate)

add_library(${TARGET_NAME} STATIC
  utils.cpp)

target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS})
