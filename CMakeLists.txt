cmake_minimum_required(VERSION 3.16)
project(roboteam_robothub)

find_package(Qt5 COMPONENTS Network REQUIRED)

add_executable(roboteam_robothub
        src/packing.cpp
        src/GRSim.cpp
        src/SerialDeviceManager.cpp
        src/RobotHub.cpp
        src/main.cpp
        src/SSLSimulator.cpp
        )


target_include_directories(roboteam_robothub
        PRIVATE include/roboteam_robothub
        INTERFACE include
        )

target_link_libraries(roboteam_robothub
        PRIVATE roboteam_proto
        PRIVATE roboteam_utils
        PRIVATE Qt5::Network
        )



#add_test(robothub_tests
#    ${SRCS}
#    test/main.cpp
#    test/packing.cpp
#    )

#target_compile_options(robothub_tests PUBLIC --coverage)


#target_link_libraries(robothub_tests ${QT_LIBRARIES} roboteam_proto gcov)
#
#include(CodeCoverage)
#APPEND_COVERAGE_COMPILER_FLAGS()
#SETUP_TARGET_FOR_COVERAGE_LCOV(NAME ${PROJECT_NAME}_coverage EXECUTABLE robothub_tests)

