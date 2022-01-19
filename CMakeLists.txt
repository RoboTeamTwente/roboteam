cmake_minimum_required(VERSION 3.16)
project(roboteam_robothub)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
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

# Generate the proto files for the simulator manager
find_package(Protobuf 3.9.1 REQUIRED)
add_subdirectory(simulation_proto)

# Create the make file for the simulatorManager library
add_library(simulator_manager STATIC
        "src/simulation/SimulatorManager.cpp"
        "src/simulation/RobotControlCommand.cpp"
        "src/simulation/ConfigurationCommand.cpp")
target_include_directories(simulator_manager PUBLIC "include")
target_link_libraries(simulator_manager PUBLIC
        simulation_manager_proto
        Qt5::Network
)

# Create the make file for the basestationManager library
add_library(basestation_manager STATIC
        "src/basestation/LibusbUtilities.cpp"
        "src/basestation/Basestation.cpp"
        "src/basestation/BasestationCollection.cpp"
        "src/basestation/BasestationManager.cpp")
target_include_directories(basestation_manager PUBLIC
        "include"
        "roboteam_embedded_messages/include")
target_link_libraries(basestation_manager PUBLIC
        Threads::Threads
        lib::usb
)

# Create the make file for RobotHub, which uses the basestationManager and simulationManager library
add_executable(roboteam_robothub "src/RobotHub.cpp")
target_include_directories(roboteam_robothub
        PRIVATE include
)
target_link_libraries(roboteam_robothub
        PRIVATE simulator_manager
        PRIVATE basestation_manager
        PRIVATE roboteam_networking
)

# Create the make file for robothub_enumerate_usb
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

# Create make file for simulation formation test
add_executable(test_formation
        test/sendFormationToSimulator.cpp
)

target_link_libraries(test_formation PUBLIC simulator_manager)