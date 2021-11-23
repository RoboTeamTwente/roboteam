cmake_minimum_required(VERSION 3.21.2)
project(roboteam_networking VERSION 1.0)

set(CMAKE_CXX_STANDARD "17")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")
include(GetDependencies)

add_library(networking_utils STATIC
    "src/utils/Channel.cpp"
    "src/utils/Channels.cpp"
    "src/utils/Publisher.cpp"
    "src/utils/Subscriber.cpp")
target_include_directories(networking_utils PUBLIC "include")
target_link_libraries(networking_utils PUBLIC stx zmqpp)