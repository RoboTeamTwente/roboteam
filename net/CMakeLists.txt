add_library(net STATIC
        src/netraw.cpp
        src/robocup_ssl_client.cpp
        )
target_include_directories(net
        PRIVATE include/net
        INTERFACE include
        )
target_link_libraries(net
        PUBLIC roboteam_proto
        )