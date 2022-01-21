cmake_minimum_required(VERSION 3.16)
project(roboteam_networking VERSION 1.0)

set(CMAKE_CXX_STANDARD "17")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")
include(GetDependencies)

# Create make file for all theproto files as target 'roboteam_networking_proto'
find_package(Protobuf 3.9.1 REQUIRED)
add_subdirectory(proto)

add_library(roboteam_networking_utils STATIC
    "src/utils/Channel.cpp"
    "src/utils/Channels.cpp")
target_include_directories(roboteam_networking_utils PUBLIC "include")
target_link_libraries(roboteam_networking_utils PUBLIC zmqpp)

add_library(roboteam_networking STATIC
    "src/RobotCommandsNetworker.cpp"
    "src/RobotFeedbackNetworker.cpp"
    "src/SettingsNetworker.cpp"
    "src/WorldNetworker.cpp"
    "src/SimulationConfigurationNetworker.cpp"
)

target_include_directories(roboteam_networking PUBLIC "include")
target_link_libraries(roboteam_networking PUBLIC roboteam_networking_utils roboteam_networking_proto)

add_subdirectory(tests)
