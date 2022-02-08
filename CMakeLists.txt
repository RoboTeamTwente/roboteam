cmake_minimum_required(VERSION 3.16)
project(rtt)

# Build type compile flags
set(CPP_STANDARD_FLAG "-std=c++20")
set(DEBUG_COMPILE_FLAGS "${CPP_STANDARD_FLAG}" "-fPIC" "-Wall" "-g" "-O0")
set(RELEASE_COMPILE_FLAGS "${CPP_STANDARD_FLAG}" "-fPIC" "-Ofast")

# Specify manually which compiler arguments we want to use, either DEBUG or RELEASE ones
# If you want to build in release mode, pass '-DCMAKE_BUILD_TYPE=RELEASE' as argument to cmake
if (CMAKE_BUILD_TYPE MATCHES RELEASE)
    message("Building in release mode")

    set(COMPILER_FLAGS "${RELEASE_COMPILE_FLAGS}")
else()
    message("Building in debug mode")

    # The default compiler flags are for debugging
    set(COMPILER_FLAGS "${DEBUG_COMPILE_FLAGS}")
endif()

# Get the targets of the dependencies
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
include(ccache) # For daster compiling
include(LocateQt5) # For GUI and some networking
include(FindSodium) # ? Not required...
include(GetSDL) # For connecting joysticks to the GUI
include(GetWebsocket) # ?
include(GetZmqpp) # For networking between AI, RobotHub and World
include(BuildLibusb) # USB library for RobotHub

# This will create a file in the build folder that contains the commands used for compiling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# internal projects
add_subdirectory(roboteam_networking)
add_subdirectory(roboteam_utils)
add_subdirectory(roboteam_ai)
add_subdirectory(roboteam_robothub)
add_subdirectory(roboteam_world)