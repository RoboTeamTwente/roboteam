cmake_minimum_required(VERSION 2.8)
project(roboteam_robothub)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_FLAGS "-g -std=c++14 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  roboteam_msgs
  rostest
  roboteam_utils
)

# This part actually makes sure the network interface
# of Qt4 is included! So while rqt uses Qt5, for our
# networking purposes we use Qt4.
# RoboTeam Twente works in mysterious ways.
set(QT_USE_QTNETWORK TRUE)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime roboteam_msgs
)

set(SRCS
    src/packing.cpp
    src/GRSim.cpp
    src/SerialDeviceManager.cpp
    src/SerialDeviceManager.h)

add_executable(roboteam_robothub
    src/main.cpp
    ${SRCS})

catkin_add_gtest(robothub_tests
    ${SRCS}
    test/main.cpp
    test/packing.cpp
    )
target_compile_options(robothub_tests PUBLIC --coverage)

target_link_libraries(roboteam_robothub 
    ${catkin_LIBRARIES} 
    ${QT_LIBRARIES}
    )

target_link_libraries(robothub_tests
        ${catkin_LIBRARIES}
        gcov)

add_dependencies(roboteam_robothub 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    )


include(CodeCoverage)
APPEND_COVERAGE_COMPILER_FLAGS()
SETUP_TARGET_FOR_COVERAGE_LCOV(NAME ${PROJECT_NAME}_coverage EXECUTABLE robothub_tests)

