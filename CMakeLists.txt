cmake_minimum_required(VERSION 3.16)
project(rtt)

# Get the targets of the dependencies
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
include(ccache) # For daster compiling
include(LocateQt5) # For GUI and some networking
include(FindSodium) # ? Not required...
include(GetSDL) # For connecting joysticks to the GUI
include(GetWebsocket) # ?
include(GetZmqpp) # For networking between AI, RobotHub and World
include(BuildLibusb) # USB library for RobotHub
include(CheckCXXCompilerFlag) # For testing compiler cxx standard support

# Testing compiler cxx support
check_cxx_compiler_flag(-std=c++20 COMPILER_SUPPORTS_CXX_20)
check_cxx_compiler_flag(-std=c++2a COMPILER_SUPPORTS_CXX_2a)

if (COMPILER_SUPPORTS_CXX_20)
    set(CPP_STANDARD_FLAG "-std=c++20")
elseif (COMPILER_SUPPORTS_2a)
    set(CPP_STANDARD_FLAG "-std=c++2a")
else ()
    set(CPP_STANDARD_FLAG "")
    # This CMake setting does not guarantee everything will be compiled with c++20 support, as
    # these settings can be overwritten in targets by other libraries
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Build type compile flags
set(DEBUG_COMPILE_FLAGS "${CPP_STANDARD_FLAG}" "-fPIC" "-Wall" "-g" "-O0")
set(RELEASE_COMPILE_FLAGS "${CPP_STANDARD_FLAG}" "-fPIC" "-Ofast")

# Specify manually which compiler arguments we want to use, either DEBUG (default) or RELEASE ones
# If you want to build in release mode, pass '-DCMAKE_BUILD_TYPE=RELEASE' as argument to cmake
if (CMAKE_BUILD_TYPE MATCHES RELEASE)
    message("Building in release mode")

    set(COMPILER_FLAGS "${RELEASE_COMPILE_FLAGS}")
else()
    message("Building in debug mode")

    # The default compiler flags are for debugging
    set(COMPILER_FLAGS "${DEBUG_COMPILE_FLAGS}")
endif()

message("Compile flags: " "${COMPILER_FLAGS}")

# This will create a file in the build folder that contains the commands used for compiling, useful for debugging
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# internal projects
add_subdirectory(roboteam_networking)
add_subdirectory(roboteam_utils)
add_subdirectory(roboteam_ai)
add_subdirectory(roboteam_robothub)
add_subdirectory(roboteam_world)

enable_testing()