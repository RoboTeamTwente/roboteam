#include(ccache)
#include(cotire)

# find the protobuf compiler and libraries
add_library(networking STATIC
        src/Channel.cpp
        src/Channels.cpp
        src/Module.cpp
        )

target_include_directories(networking
        PUBLIC include
        )

target_link_libraries(networking
        PUBLIC stx
        PUBLIC roboteam_proto
        # since Publishers/subscribers are templates these 2 libraries need to be PUBLIC
        PUBLIC lib::zmq
        PUBLIC lib::zmqpp
        )
