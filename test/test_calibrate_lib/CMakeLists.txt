
set(TARGET_NAME test_calibrate_lib)

add_executable(${TARGET_NAME}
  main.cpp)

target_link_libraries(${TARGET_NAME}
  CommandLine
  Calibrate)

install(TARGETS ${TARGET_NAME}
  RUNTIME DESTINATION tests)

add_test(
  NAME calibrate_lib
  CONFIGURATIONS Release Debug
  WORKING_DIRECTORY ${TESTS_WD}
  COMMAND ${TARGET_NAME})
