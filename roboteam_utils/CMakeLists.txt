cmake_minimum_required(VERSION 3.22)
project(roboteam_utils VERSION 0.1 DESCRIPTION "utils for Roboteam Twente")

file(GLOB_RECURSE ROBOTEAM_UTILS_SRC
        "${PROJECT_SOURCE_DIR}/src/utils/*.cpp"
)

file(GLOB_RECURSE ROBOTEAM_UTILS_TEST_SRC
    "${PROJECT_SOURCE_DIR}/test/utils/*.cpp"
)
add_library(roboteam_utils STATIC
        ${ROBOTEAM_UTILS_SRC}
)

target_include_directories(roboteam_utils
        PRIVATE include/roboteam_utils
        PUBLIC include
)

target_link_libraries(roboteam_utils
        PRIVATE roboteam_networking
)
target_compile_options(roboteam_utils PRIVATE "${COMPILER_FLAGS}")

# -- TESTS --

add_executable(roboteam_utils_tests
        ${ROBOTEAM_UTILS_TEST_SRC}
)

target_link_libraries(roboteam_utils_tests
        PRIVATE roboteam_utils
        PRIVATE GTest::gtest
        PRIVATE GTest::gtest_main
)

target_compile_options(roboteam_utils_tests PRIVATE "${COMPILER_FLAGS}")

gtest_discover_tests(roboteam_utils_tests)