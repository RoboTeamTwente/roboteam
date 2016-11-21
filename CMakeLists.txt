cmake_minimum_required(VERSION 2.8)
project(roboteam_robothub)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

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

#find_package(Protobuf REQUIRED)
#include_directories(${PROTOBUF_INCLUDE_DIRS})

#protobuf_generate_cpp(PROTO_CPP PROTO_H
#    src/proto/grSim_Replacement.proto
#    src/proto/grSim_Commands.proto
#    src/proto/grSim_Packet.proto
#)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)

#find_package(message_generation)

#generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime roboteam_msgs
)

add_executable(roboteam_robothub src/main.cpp)
target_link_libraries(roboteam_robothub ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(roboteam_robothub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(input src/input_example.cpp)
target_link_libraries(input ${catkin_LIBRARIES})
add_dependencies(input ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(robothub_test
    test/main.cpp
    test/packing_test.cpp
    test/comm_test.cpp

    src/packing.cpp
    )
target_link_libraries(robothub_test ${catkin_LIBRARIES})
