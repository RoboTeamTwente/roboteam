cmake_minimum_required(VERSION 3.10)
project(roboteam_world)

find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Network)
add_subdirectory(data)
add_subdirectory(net)
add_subdirectory(observer)

add_executable(roboteam_observer
        src/main.cpp
        src/Handler.cpp
        )
target_link_libraries(roboteam_observer
        PRIVATE observer
        PRIVATE net
        PRIVATE roboteam_proto
        PRIVATE roboteam_utils
        PRIVATE Qt5::Network
        )
target_include_directories(roboteam_observer
        PRIVATE include/roboteam_observer
        INTERFACE include
        )

set_target_properties(roboteam_observer PROPERTIES
        CMAKE_CXX_STANDARD 20
        CMAKE_CXX_STANDARD_REQUIRED 1
        )