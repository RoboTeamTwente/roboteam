cmake_minimum_required(VERSION 3.12)
project(roboteam_utils VERSION 0.1 DESCRIPTION "utils for Roboteam Twente")

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")
include(cotire)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(ENABLE_DEBUG_GRAPHICS TRUE)

if (ENABLE_DEBUG_GRAPHICS)
    add_definitions(-DRTT_CMAKE_ENABLE_DEBUG_GRAPHICS=1)
else (ENABLE_DEBUG_GRAPHICS)
    add_definitions(-DRTT_CMAKE_ENABLE_DEBUG_GRAPHICS=0)
endif (ENABLE_DEBUG_GRAPHICS)

find_package(Protobuf REQUIRED)

set(CMAKE_CXX_FLAGS "\
--std=c++17 ${CMAKE_CXX_FLAGS} \
-Wall \
-Wno-unused-function \
-Wsign-compare \
-g \
"
        )


include_directories(${CMAKE_CURRENT_LIST_DIR}/include/roboteam_utils)

add_library(roboteam_utils
        src/Polygon.cpp
        src/LineBase.cpp
        src/Line.cpp
        src/LineSegment.cpp
        src/Vector2.cpp
        src/Angle.cpp
        src/Position.cpp
      #  src/LastWorld.cpp
        src/Mathematics.cpp
        src/Cone.cpp
        src/normalize.cpp
        src/Clip.cpp
        src/Optimization.cpp
        src/Arc.cpp
        src/Section.cpp )


target_link_libraries(roboteam_utils roboteam_proto)
