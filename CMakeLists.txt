cmake_minimum_required(VERSION 3.16)
project(roboteam_robothub)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

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
        src/simulation/SimulatorManager.cpp
        src/simulation/RobotControlCommand.cpp
        src/simulation/ConfigurationCommand.cpp
        src/basestation/Packing.cpp
        src/basestation/LibusbUtilities.cpp
        src/basestation/BasestationManager.cpp
        src/RobotHub.cpp
)
target_include_directories(roboteam_robothub
        PRIVATE include
        PRIVATE roboteam_embedded_messages/include
)
target_link_libraries(roboteam_robothub
        PRIVATE roboteam_proto
        PRIVATE roboteam_utils
        PRIVATE Qt5::Network
        Threads::Threads
        lib::usb
)

add_executable(roboteam_robothub_enumerate_usb
        src/basestation/LibusbUtilities.cpp
        src/basestation/EnumerateBasestations.cpp
)
target_include_directories(roboteam_robothub_enumerate_usb
        PRIVATE include/
)
target_link_libraries(roboteam_robothub_enumerate_usb
        Threads::Threads
        lib::usb
)