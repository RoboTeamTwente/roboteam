cmake_minimum_required(VERSION 3.16)
project(rtt)

set(CMAKE_UNITY_BUILD ON)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

set(CMAKE_GENERATOR Ninja)

include(ccache)

set(CMAKE_CXX_STANDARD 20) # -std=c++17
set(CMAKE_CXX_STANDARD_REQUIRED ON) # -std=c++17 required
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # -fPic

set(CMAKE_WARN_DEPRECATED FALSE)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

include(LocateQt5)
include(FindSodium)
include(GetDependencies)

# internal projects
add_subdirectory(roboteam_proto)
add_subdirectory(roboteam_utils)
add_subdirectory(roboteam_ai)
add_subdirectory(roboteam_robothub)
add_subdirectory(roboteam_world)
add_subdirectory(roboteam_central_server)
