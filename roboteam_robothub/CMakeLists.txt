cmake_minimum_required(VERSION 3.22)
project(roboteam_robothub)

# for MacOS X or iOS, watchOS, tvOS (since 3.10.3)
if(APPLE)
    set(Qt5Network_DIR "/usr/local/opt/qt/lib/cmake/Qt5Network")
endif()

# for Linux, BSD, Solaris, Minix
if(UNIX AND NOT APPLE)
    SET(Qt5Network_DIR "/usr/include/x86_64-linux-gnu/qt5/Qt5Network")
endif()

find_package(Qt5Network REQUIRED)

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
        roboteam_utils
)
target_compile_options(simulator_manager PRIVATE "${COMPILER_FLAGS}")

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
        roboteam_utils
)
target_compile_options(basestation_manager PRIVATE "${COMPILER_FLAGS}")

# Create the make file for RobotHub, which uses the basestationManager and simulationManager library
add_executable(roboteam_robothub "src/RobotHubStatistics.cpp" "src/RobotHub.cpp")
target_include_directories(roboteam_robothub
        PRIVATE include
)
target_link_libraries(roboteam_robothub
        PRIVATE simulator_manager
        PRIVATE basestation_manager
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
)
target_compile_options(roboteam_robothub PRIVATE "${COMPILER_FLAGS}")

# add_executable(roboteam_robothub_logger "src/RobotHubLogger.cpp")
# target_include_directories(roboteam_robothub_logger
#         PRIVATE include
#         "roboteam_embedded_messages/include"
# )
# target_link_libraries(roboteam_robothub_logger
#         PRIVATE roboteam_networking
#         PRIVATE roboteam_utils
# )
# target_compile_options(roboteam_robothub_logger PRIVATE "${COMPILER_FLAGS}")

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
target_compile_options(roboteam_robothub_enumerate_usb PRIVATE "${COMPILER_FLAGS}")

# Create the make file for scripts/RobotControl.cpp
add_executable(roboteam_robothub_robotControlScript
        "scripts/RobotControlScript.cpp"
)
target_link_libraries(roboteam_robothub_robotControlScript PRIVATE roboteam_networking)
target_compile_options(roboteam_robothub_robotControlScript PRIVATE "${COMPILER_FLAGS}")