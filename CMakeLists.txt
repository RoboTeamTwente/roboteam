cmake_minimum_required(VERSION 3.16.0)
project(roboteam_interface VERSION 1.0.0 LANGUAGES CXX)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.10.5/json.tar.xz)
FetchContent_MakeAvailable(json)


add_subdirectory(libs)
add_subdirectory(apps)
