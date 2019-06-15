set(TESTS_WD ${CMAKE_CURRENT_BINARY_DIR})
set(INPUTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Inputs)
add_definitions(-DINPUTS_DIR=${INPUTS_DIR})

add_subdirectory(test_calibrate_lib)

# add_test(
#   NAME basic_acceptance_1
#   CONFIGURATIONS Release Debug
#   WORKING_DIRECTORY ${TESTS_WD}
#   COMMAND stitch ${INPUTS_DIR}/1a.jpg ${INPUTS_DIR}/1b.jpg -b=3x4)
#
# add_test(
#   NAME basic_acceptance_2
#   CONFIGURATIONS Release Debug
#   WORKING_DIRECTORY ${TESTS_WD}
#   COMMAND stitch ${INPUTS_DIR}/3a.jpg ${INPUTS_DIR}/3b.jpg ${INPUTS_DIR}/3c.jpg -b=5x3)
