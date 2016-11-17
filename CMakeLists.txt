cmake_minimum_required(VERSION 2.8.3)
project(roboteam_utils)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roboteam_msgs)

set(CMAKE_CXX_FLAGS "--std=c++11 -Wall ${CMAKE_CXX_FLAGS}")



find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})


protobuf_generate_cpp(PROTO_CPP PROTO_H
    src/proto/grSim_Replacement.proto
    src/proto/grSim_Commands.proto
    src/proto/grSim_Packet.proto
    src/proto/messages_robocup_ssl_detection.proto
    src/proto/messages_robocup_ssl_geometry.proto
    src/proto/messages_robocup_ssl_geometry_legacy.proto
    src/proto/messages_robocup_ssl_referee.proto
    src/proto/messages_robocup_ssl_wrapper.proto
    src/proto/messages_robocup_ssl_wrapper_legacy.proto
)


protobuf_generate_python(PROTO_PY
    src/proto/grSim_Replacement.proto
    src/proto/grSim_Commands.proto
    src/proto/grSim_Packet.proto
    src/proto/messages_robocup_ssl_detection.proto
    src/proto/messages_robocup_ssl_geometry.proto
    src/proto/messages_robocup_ssl_geometry_legacy.proto
    src/proto/messages_robocup_ssl_referee.proto
    src/proto/messages_robocup_ssl_wrapper.proto
    src/proto/messages_robocup_ssl_wrapper_legacy.proto
)



catkin_package(
  INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR} # Allows for including of generated protobuf header files.
  LIBRARIES roboteam_utils roboteam_utils_proto
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ library
# add_library(roboteam_utils
#   src/${PROJECT_NAME}/roboteam_utils.cpp
# )

add_library(roboteam_utils src/Vector2.cpp src/Position.cpp)
add_dependencies(roboteam_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roboteam_utils ${catkin_LIBRARIES})

# Protobuf messages.
add_library(roboteam_utils_proto ${PROTO_CPP} ${PROTO_H})
add_dependencies(roboteam_utils_proto ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roboteam_utils_proto ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} ${PROTOBUF_LIBRARY})

catkin_add_gtest(utils_test 
    test/main.cpp 
    test/Vector2Test.cpp
    test/PositionTest.cpp
    test/GraphTest.cpp
    src/Vector2.cpp
    src/Position.cpp
)
target_link_libraries(utils_test ${catkin_LIBRARIES})
