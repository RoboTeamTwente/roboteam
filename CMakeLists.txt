cmake_minimum_required(VERSION 3.16)
project(roboteam_utils VERSION 0.1 DESCRIPTION "utils for Roboteam Twente")

add_subdirectory(networking)

add_library(roboteam_utils STATIC
        src/Polygon.cpp
        src/Line.cpp
        src/LineSegment.cpp
        src/Vector2.cpp
        src/Angle.cpp
        src/Position.cpp
        src/Mathematics.cpp
        src/normalize.cpp
        src/Hungarian.cpp
        src/Arc.cpp
        src/Timer.cpp
        src/Rectangle.cpp
        src/Circle.cpp
        src/pid.cpp
        src/Tube.cpp
        src/Time.cpp
        src/Random.cpp
        src/HalfLine.cpp
        src/Grid.cpp
        src/Shadow.cpp
        )


target_include_directories(roboteam_utils
        PRIVATE include/roboteam_utils
        INTERFACE include
        )

target_link_libraries(roboteam_utils
        PUBLIC roboteam_proto
        PUBLIC networking
        )

add_executable(utils_tests
        test/main.cpp
        test/AngleTest.cpp
        test/ArcTest.cpp
        test/LineTest.cpp
        test/LineIntersectionTest.cpp
        test/LineProjectionTest.cpp
        test/CircleTest.cpp
        test/normaliseTest.cpp
        test/PolygonTest.cpp
        test/PositionTest.cpp
        test/Vector2Test.cpp
        test/RectangleTest.cpp
        test/HungarianTest.cpp
        test/TubeTest.cpp
        test/TimeTest.cpp
        test/RandomTest.cpp
        test/StateMachineTest.cpp
        test/GridTest.cpp
        test/StaticVectorTest.cpp
        test/ShadowTest.cpp
        test/CircularBufferTest.cpp
        test/PubSubTest.cpp
        )

target_link_libraries(utils_tests
        PRIVATE roboteam_utils
        PRIVATE gtest
        )
