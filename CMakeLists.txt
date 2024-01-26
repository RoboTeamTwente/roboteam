cmake_minimum_required(VERSION 3.22)
project(rtt)

# Check for Release or Debug build
if (CMAKE_BUILD_TYPE MATCHES RELEASE OR CMAKE_BUILD_TYPE MATCHES Release)
    set(RTT_BUILD_RELEASE TRUE)
    set(RTT_BUILD_FOLDER release)
else()
    set(RTT_BUILD_RELEASE FALSE)
    set(RTT_BUILD_FOLDER debug)
endif()

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${RTT_BUILD_FOLDER}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${RTT_BUILD_FOLDER}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${RTT_BUILD_FOLDER}/lib)

# Get the targets of the dependencies
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
include(BuildLibusb) # USB library for RobotHub

# Enable C++ Modules support for Cmake
set(CMAKE_EXPERIMENTAL_CXX_MODULE_CMAKE_API 2182bf5c-ef0d-489a-91da-49dbc3090d2a)
set(CMAKE_EXPERIMENTAL_CXX_MODULE_DYNDEP 1)

# Testing compiler cxx support
# check_cxx_compiler_flag(-std=c++20 COMPILER_SUPPORTS_CXX_20)
# check_cxx_compiler_flag(-std=c++2a COMPILER_SUPPORTS_CXX_2a)

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
endif ()

# Build type compile flags
set(DEBUG_COMPILE_FLAGS "${CPP_STANDARD_FLAG}" "-fPIC" "-g" "-O0" "-Wall" "-Wextra" "-Wnon-virtual-dtor" "-pedantic" "-fdiagnostics-color=always")

# march mtune https://stackoverflow.com/questions/54039176/mtune-and-march-when-compiling-in-a-docker-image
set(RELEASE_COMPILE_FLAGS "${CPP_STANDARD_FLAG}" "-fPIC" "-Ofast" "-march=x86-64" "-mtune=generic" "-fdiagnostics-color=always")

# Specify manually which compiler arguments we want to use, either DEBUG (default) or RELEASE ones
# If you want to build in release mode, pass '-DCMAKE_BUILD_TYPE=RELEASE' as argument to cmake
if (RTT_BUILD_RELEASE)
    message("Building in Release mode")
    set(COMPILER_FLAGS "${RELEASE_COMPILE_FLAGS}")
else ()
    message("Building in Debug mode")
    set(COMPILER_FLAGS "${DEBUG_COMPILE_FLAGS}")
endif()

message("Compile flags: " "${COMPILER_FLAGS}")

# This will create a file in the build folder that contains the commands used for compiling, useful for debugging
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (APPLE)
    include_directories("/usr/local/include")     # macOS x86 bottles (includes rosette2)
    include_directories("/opt/homebrew/include")  # Apple silicon bottles
endif()