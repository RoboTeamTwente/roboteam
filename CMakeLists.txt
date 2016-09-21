cmake_minimum_required(VERSION 2.8)
project(roboteam_robothub)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(libs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roboteam_msgs
)

set(QT_USE_QTNETWORK TRUE)
find_package(Qt REQUIRED)
include(${QT_USE_FILE})
list(APPEND libs ${QT_LIBRARIES})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
list(APPEND libs ${PROTOBUF_LIBRARIES})

#add_message_files(
#	FILES 
#	robot_command.msg
#)

protobuf_generate_cpp(PROTO_CPP PROTO_H
    src/grSim_Replacement.proto
    src/grSim_Commands.proto
    src/grSim_Packet.proto
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sender ${PROTO_CPP} ${PROTO_H} src/sender.cpp)
target_link_libraries(sender ${catkin_LIBRARIES} ${libs})
add_dependencies(sender roboteam_msgs_generate_messages_cpp)

add_executable(input src/input_example.cpp)
target_link_libraries(input ${catkin_LIBRARIES})

#find_package(message_generation)

#generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime roboteam_msgs
)
