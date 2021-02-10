cmake_minimum_required(VERSION 3.10)
project(roboteam_robothub)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3")

# for MacOS X or iOS, watchOS, tvOS (since 3.10.3)
if(APPLE)
    set(Qt5Network_DIR "/usr/local/opt/qt/lib/cmake/Qt5Network")
endif()

# for Linux, BSD, Solaris, Minix
if(UNIX AND NOT APPLE)
    SET(Qt5Network_DIR "/usr/include/x86_64-linux-gnu/qt5/Qt5Network")
endif()

find_package(Qt5Network REQUIRED)

add_executable(roboteam_robothub
        src/packing.cpp
        src/GRSim.cpp
        src/SerialDeviceManager.cpp
        src/RobotHub.cpp
        src/main.cpp)


target_include_directories(roboteam_robothub
        INTERFACE include
        PRIVATE include/roboteam_robothub
        PRIVATE src )

target_link_libraries(roboteam_robothub
        PUBLIC roboteam_proto
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

