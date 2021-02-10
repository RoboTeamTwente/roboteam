add_library(data STATIC
        src/RobotParameters.cpp
        )
target_include_directories(data
        INTERFACE include
        PRIVATE include/data
        )
target_link_libraries(data
        PUBLIC roboteam_proto)

