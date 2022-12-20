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
        src/utils/Hungarian.cpp
        src/utils/Arc.cpp
        src/utils/Timer.cpp
        src/utils/LazyRectangle.cpp
        src/utils/Circle.cpp
        src/utils/pid.cpp
        src/utils/Tube.cpp
        src/utils/Time.cpp
        src/utils/Random.cpp
        src/utils/HalfLine.cpp
        src/utils/Grid.cpp
        src/utils/Shadow.cpp
        src/utils/Polynomial.cpp
        src/utils/RobotShape.cpp
        src/utils/RobotCommands.cpp
        src/utils/RobotFeedback.cpp
        src/utils/FileLogger.cpp
        src/utils/Teams.cpp
        src/utils/AIData.cpp
        src/utils/FieldRectangle.cpp
        src/utils/Field.cpp
        src/utils/Robot.cpp
        src/utils/World.cpp
        src/utils/Ball.cpp
        )

set(ROBOTEAM_UTILS_TEST_SRC
        test/utils/AngleTest.cpp
        test/utils/ArcTest.cpp
        test/utils/LineTest.cpp
        test/utils/LineIntersectionTest.cpp
        test/utils/LineProjectionTest.cpp
        test/utils/LineSegmentTest.cpp
        test/utils/CircleTest.cpp
        test/utils/PolygonTest.cpp
        test/utils/PositionTest.cpp
        test/utils/Vector2Test.cpp
        test/utils/LazyRectangleTest.cpp
        test/utils/HungarianTest.cpp
        test/utils/TubeTest.cpp
        test/utils/TimeTest.cpp
        test/utils/RandomTest.cpp
        test/utils/StateMachineTest.cpp
        test/utils/GridTest.cpp
        test/utils/StaticVectorTest.cpp
        test/utils/ShadowTest.cpp
        test/utils/CircularBufferTest.cpp
        test/utils/RobotShapeTest.cpp
        test/utils/RobotCommandsTest.cpp
        test/utils/RobotFeedbackTest.cpp
        )

add_library(roboteam_utils STATIC
        ${ROBOTEAM_UTILS_SRC}
        )


target_include_directories(roboteam_utils
        PRIVATE include/roboteam_utils
        PUBLIC include
        )

target_compile_options(roboteam_utils PRIVATE "${COMPILER_FLAGS}")

# -- TESTS --

add_executable(roboteam_utils_tests
        ${ROBOTEAM_UTILS_TEST_SRC}
        )

target_link_libraries(roboteam_utils_tests
        PRIVATE roboteam_utils
        PRIVATE GTest::gtest
        PRIVATE GTest::gtest_main
        PRIVATE pthread
        )

target_compile_options(roboteam_utils_tests PRIVATE "${COMPILER_FLAGS}")

gtest_discover_tests(roboteam_utils_tests)
