cmake_minimum_required(VERSION 3.10)
project(roboteam_utils VERSION 0.1 DESCRIPTION "utils for Roboteam Twente")

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")
include(cotire)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

add_library(roboteam_utils STATIC
        src/Polygon.cpp
        include/roboteam_utils/Polygon.h
        src/Line.cpp
        include/roboteam_utils/Line.h
        src/LineSegment.cpp
        include/roboteam_utils/LineSegment.h
        src/Vector2.cpp
        include/roboteam_utils/Vector2.h
        src/Angle.cpp
        include/roboteam_utils/Angle.h
        src/Position.cpp
        include/roboteam_utils/Position.h
        src/Mathematics.cpp
        include/roboteam_utils/Mathematics.h
        #        src/Cone.cpp
        #        include/roboteam_utils/Cone.h
        src/normalize.cpp
        include/roboteam_utils/normalize.h
        src/Clip.cpp
        include/roboteam_utils/Hungarian.h
        src/Hungarian.cpp
        include/roboteam_utils/Clip.h
        src/Optimization.cpp
        include/roboteam_utils/Optimization.h
        src/Arc.cpp
        include/roboteam_utils/Arc.h
        src/Timer.cpp
        include/roboteam_utils/Timer.h
        include/roboteam_utils/type_traits.h
        #include/roboteam_utils/BiMap.h
        #include/roboteam_utils/Graph.h
        include/roboteam_utils/Definitions.h
        include/roboteam_utils/Statistics.h
        include/roboteam_utils/Print.h
        src/Rectangle.cpp
        include/roboteam_utils/Rectangle.h
        src/Circle.cpp
        include/roboteam_utils/Circle.h
        src/pid.cpp
        include/roboteam_utils/pid.h
        src/Triangle.cpp
        include/roboteam_utils/Triangle.h
        src/Tube.cpp
        include/roboteam_utils/Tube.h
        )



target_include_directories(roboteam_utils
        INTERFACE include
        PRIVATE include/roboteam_utils
        PRIVATE src)

target_link_libraries(roboteam_utils
        PUBLIC roboteam_proto
        )
target_compile_options(roboteam_utils PRIVATE
        --std=c++17
        -Wall
#        -Werror
        -Wextra
        -Wno-unused-function
        -Wno-comment
        -Wsign-compare
        -g
        )
add_executable(utils_tests
        test/main.cpp
        test/ArcTest.cpp
        #test/BiMapTest.cpp
        #test/GraphTest.cpp
        test/LineTest.cpp
        test/CircleTest.cpp
        test/normaliseTest.cpp
        test/OptimizationTest.cpp
        test/PolygonTest.cpp
        test/PositionTest.cpp
        test/Vector2Test.cpp
        test/RectangleTest.cpp
        test/HungarianTest.cpp
        test/TriangleTest.cpp
        test/TubeTest.cpp
        )

if (UNIX AND NOT APPLE)
    target_link_libraries(utils_tests
            PRIVATE roboteam_utils
            PRIVATE gtest
            )
endif ()

if (APPLE)
    target_link_libraries(utils_tests
            PRIVATE roboteam_utils
            PRIVATE /usr/local/lib/libgtest.a
            PRIVATE /usr/local/lib/libgtest_main.a
            )
endif ()