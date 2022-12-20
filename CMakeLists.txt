cmake_minimum_required(VERSION 3.16)
project(rtt)

set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})

set(${CMAKE_PROJECT_NAME}_CONANFILE ${CMAKE_CURRENT_SOURCE_DIR}/conanfile.txt)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
include(conan_config)
include(ccache) # Compiler caching
include(CheckCXXCompilerFlag) # For testing compiler cxx standard support

find_package(Protobuf 3.9.1 REQUIRED)
find_package(zmqpp 4.2.0 REQUIRED)
find_package(libusb 1.0.26 REQUIRED)
find_package(SDL2 2.24.1 REQUIRED)

set(CMAKE_AUTOMOC ON) # Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 5.15.7 COMPONENTS
Gui Widgets Core Charts Network REQUIRED)

enable_testing()
find_package(GTest 1.12.1 REQUIRED)
include(GoogleTest)

message("Compile flags: " "${COMPILER_FLAGS}")

# This will create a file in the build folder that contains the commands used for compiling, useful for debugging
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 1)

if(DEFINED TRACY_ENABLE)
    message("Tracy Profiler is enabled")
else()
    message("Tracy Profiler is disabled")
endif()

# internal projects
add_subdirectory(roboteam_networking)
add_subdirectory(roboteam_utils)
add_subdirectory(roboteam_ai)
add_subdirectory(roboteam_robothub)
add_subdirectory(roboteam_world)
add_subdirectory(roboteam_interface)
