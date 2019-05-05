
set(TARGET_NAME calibrate)

add_executable(${TARGET_NAME}
  main.cpp)

target_link_libraries(${TARGET_NAME}
  CommandLine
  Calibrate)

install(TARGETS ${TARGET_NAME}
  RUNTIME DESTINATION tools)
