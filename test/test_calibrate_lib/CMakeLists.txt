
set(TARGET_NAME test_calibrate_lib)

add_executable(${TARGET_NAME}
  main.cpp)

target_include_directories(${TARGET_NAME} PRIVATE
  ${CMAKE_SOURCE_DIR}/external/googletest/googletest/include)

target_link_libraries(${TARGET_NAME}
  CommandLine
  Calibrate
  gtest)

install(TARGETS ${TARGET_NAME}
  RUNTIME DESTINATION tests)

add_test(
  NAME calibrate_lib
  CONFIGURATIONS Release Debug
  WORKING_DIRECTORY ${TESTS_WD}
  COMMAND ${TARGET_NAME})
