cmake_minimum_required(VERSION 3.16)
project(roboteam_utils VERSION 0.1 DESCRIPTION "utils for Roboteam Twente")

set(ROBOTEAM_UTILS_SRC
        src/utils/Polygon.cpp
        src/utils/Line.cpp
        src/utils/LineSegment.cpp
        src/utils/Vector2.cpp
        src/utils/Angle.cpp
        src/utils/Position.cpp
        src/utils/Mathematics.cpp
        src/utils/normalize.cpp
        src/utils/Hungarian.cpp
        src/utils/Arc.cpp
        src/utils/Timer.cpp
        src/utils/Rectangle.cpp
        src/utils/Circle.cpp
        src/utils/pid.cpp
        src/utils/Tube.cpp
        src/utils/Time.cpp
        src/utils/Random.cpp
        src/utils/HalfLine.cpp
        src/utils/Grid.cpp
        src/utils/Shadow.cpp
        )

set(ROBOTEAM_UTILS_TEST_SRC
        test/utils/main.cpp
        test/utils/AngleTest.cpp
        test/utils/ArcTest.cpp
        test/utils/LineTest.cpp
        test/utils/LineIntersectionTest.cpp
        test/utils/LineProjectionTest.cpp
        test/utils/CircleTest.cpp
        test/utils/normaliseTest.cpp
        test/utils/PolygonTest.cpp
        test/utils/PositionTest.cpp
        test/utils/Vector2Test.cpp
        test/utils/RectangleTest.cpp
        test/utils/HungarianTest.cpp
        test/utils/TubeTest.cpp
        test/utils/TimeTest.cpp
        test/utils/RandomTest.cpp
        test/utils/StateMachineTest.cpp
        test/utils/GridTest.cpp
        test/utils/StaticVectorTest.cpp
        test/utils/ShadowTest.cpp
        test/utils/CircularBufferTest.cpp
        test/utils/PubSubTest.cpp
        )

add_library(roboteam_utils STATIC
        ${ROBOTEAM_UTILS_SRC}
        )


target_include_directories(roboteam_utils
        PRIVATE include/roboteam_utils
        INTERFACE include
        )

target_link_libraries(roboteam_utils
        PUBLIC roboteam_proto
        PUBLIC utils_networking_lib
        )

add_executable(utils_tests
        ${ROBOTEAM_UTILS_TEST_SRC}
        )

target_link_libraries(utils_tests
        PRIVATE roboteam_utils
        PRIVATE gtest
        )

# -- NETWORKING --

add_library(utils_networking_lib STATIC
        src/networking/Channel.cpp
        src/networking/Channels.cpp
        src/networking/Module.cpp
        )

target_include_directories(utils_networking_lib
        PUBLIC include
        PRIVATE include/networking
        )

target_link_libraries(utils_networking_lib
        PUBLIC stx
        PUBLIC roboteam_proto
        PUBLIC zmqpp
        )
