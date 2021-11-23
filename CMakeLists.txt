cmake_minimum_required(VERSION 3.21.2)
project(roboteam_networking VERSION 1.0)

set(CMAKE_CXX_STANDARD "17")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")
include(GetDependencies)

# Create make file for all theproto files as target 'networking_proto'
find_package(Protobuf 3.9.1 REQUIRED)
add_subdirectory(proto)

add_library(networking_utils STATIC
    "src/utils/Channel.cpp"
    "src/utils/Channels.cpp")
target_include_directories(networking_utils PUBLIC "include")
target_link_libraries(networking_utils PUBLIC zmqpp)

add_library(networking STATIC
    "src/RobotCommandsNetworker.cpp"
)

target_include_directories(networking PUBLIC "include")
target_link_libraries(networking PUBLIC networking_utils networking_proto)

add_subdirectory(tests)