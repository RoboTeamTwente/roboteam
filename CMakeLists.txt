cmake_minimum_required(VERSION 3.10)
project(roboteam_robothub)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

include(ccache)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3")

# for MacOS X or iOS, watchOS, tvOS (since 3.10.3)
if(APPLE)
    set(Qt5Network_DIR "/usr/local/opt/qt/lib/cmake/Qt5Network")
endif()

# for Linux, BSD, Solaris, Minix
if(UNIX AND NOT APPLE)
    SET(Qt5Network_DIR "/usr/include/x86_64-linux-gnu/qt5/Qt5Network")
endif()

find_package(Qt5Network REQUIRED)

add_executable(roboteam_robothub
        src/packing.cpp
        src/GRSim.cpp
        src/SerialDeviceManager.cpp
        src/RobotHub.cpp)

target_include_directories(roboteam_robothub
        INTERFACE include
        PRIVATE include/roboteam_robothub
        PRIVATE src
        PRIVATE roboteam_embedded_messages/include )

target_link_libraries(roboteam_robothub
        PUBLIC roboteam_proto
        PRIVATE roboteam_utils
        PRIVATE Qt5::Network
        )

find_package(Threads)
add_executable(basestation
        src/baseStationDevelopment.cpp
        )
target_include_directories(basestation
        PRIVATE include/roboteam_robothub
        PRIVATE roboteam_embedded_messages/include
        )
target_link_libraries(basestation
        ${CMAKE_THREAD_LIBS_INIT}
        usb-1.0)


add_executable(circbuf
        src/CircularBuffer.c
        )
target_include_directories(circbuf
        PRIVATE include/roboteam_robothub
        )