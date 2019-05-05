
set(TARGET_NAME "stitch")

add_executable(${TARGET_NAME}
  main.cpp)

target_link_libraries(${TARGET_NAME}
  CommandLine
  Calibrate
  ${OpenCV_LIBS})

install(TARGETS ${TARGET_NAME}
  RUNTIME DESTINATION tools)
