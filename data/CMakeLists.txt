cmake_minimum_required(VERSION 3.16)

add_library(data STATIC
        src/RobotParameters.cpp
        )
target_include_directories(data
        PRIVATE include/data
        INTERFACE include
        )
target_link_libraries(data
        PUBLIC roboteam_proto)

