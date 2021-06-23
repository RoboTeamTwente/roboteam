cmake_minimum_required(VERSION 3.10)
project(roboteam_robothub)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

include(ccache)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O0")

# for MacOS X or iOS, watchOS, tvOS (since 3.10.3)
if(APPLE)
    set(Qt5Network_DIR "/usr/local/opt/qt/lib/cmake/Qt5Network")
endif()

# for Linux, BSD, Solaris, Minix
if(UNIX AND NOT APPLE)
    SET(Qt5Network_DIR "/usr/include/x86_64-linux-gnu/qt5/Qt5Network")
endif()

find_package(Qt5Network REQUIRED)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
add_executable(roboteam_robothub
        src/RobotHub.cpp
        src/GRSim.cpp
        src/BasestationReader.cpp
        src/Packing.cpp
        src/LibusbUtilities.cpp)
target_include_directories(roboteam_robothub
        PRIVATE include/roboteam_robothub
        PRIVATE roboteam_embedded_messages/include
        )
target_link_libraries(roboteam_robothub
        PUBLIC roboteam_proto
        PRIVATE roboteam_utils
        PRIVATE Qt5::Network
        Threads::Threads
        usb-1.0)
