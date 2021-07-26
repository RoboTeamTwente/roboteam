cmake_minimum_required(VERSION 3.16)

# find the protobuf compiler and libraries
add_library(networking STATIC
        src/Channel.cpp
        src/Channels.cpp
        src/Module.cpp
        )

target_include_directories(networking
        PUBLIC include
        PRIVATE include/networking
        )

target_link_libraries(networking
        PUBLIC stx
        PRIVATE roboteam_proto
        # since Publishers/subscribers are templates these 2 libraries need to be PUBLIC
        PUBLIC zmqpp
        )
