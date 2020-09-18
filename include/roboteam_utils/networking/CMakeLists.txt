#include(ccache)
#include(cotire)

# find the protobuf compiler and libraries
find_package(zeroMQ REQUIRED)
find_package(zmqpp 4.1.2 REQUIRED)

add_library(networking STATIC
        src/Channel.cpp
        src/Channels.cpp
        )

target_include_directories(networking
        PUBLIC ./include
        )


target_link_libraries(networking
        # since Publishers/subscribers are templates these 2 libraries need to be PUBLIC
        PUBLIC ${ZeroMQ_LIBRARIES}
        PUBLIC ${ZMQPP_LIBRARIES}

        PUBLIC roboteam_proto
        )