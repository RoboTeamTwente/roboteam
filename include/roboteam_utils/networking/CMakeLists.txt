#include(ccache)
#include(cotire)

# find the protobuf compiler and libraries
find_package(zeroMQ REQUIRED)
find_package(zmqpp 4.1.2 REQUIRED)

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
        PUBLIC ${ZMQPP_LIBRARIES}
        PUBLIC ${ZeroMQ_LIBRARIES}
        )
