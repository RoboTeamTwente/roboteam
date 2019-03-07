cmake_minimum_required(VERSION 2.8)
project(roboteam_robothub)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

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
find_package(Qt REQUIRED)
include(${QT_USE_FILE})

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime roboteam_msgs
)

add_executable(roboteam_robothub 
    src/main.cpp
    src/packing.cpp
    src/GRSim.cpp
)

target_link_libraries(roboteam_robothub 
    ${catkin_LIBRARIES} 
    ${QT_LIBRARIES}
    )
add_dependencies(roboteam_robothub 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    )

