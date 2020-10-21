cmake_minimum_required(VERSION 3.10)
project(roboteam_world)

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")
include(cotire)
include(ccache)
find_package(Eigen3 REQUIRED)

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
        )
target_include_directories(roboteam_observer
        PRIVATE include/roboteam_observer
        INTERFACE include
        )

target_compile_options(roboteam_observer PRIVATE
        -std=c++17
        -O3
)
