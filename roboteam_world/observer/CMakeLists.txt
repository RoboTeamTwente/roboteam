add_library(observer STATIC
        src/filters/Scaling.h
        src/Observer.cpp
        src/parameters/RobotParameterDatabase.cpp
        src/filters/referee/RefereeFilter.cpp
        src/filters/vision/CameraObjectFilter.cpp
        src/filters/vision/robot/CameraRobotFilter.cpp
        src/filters/vision/ball/CameraGroundBallFilter.cpp
        src/filters/vision/WorldFilter.cpp
        src/filters/vision/GeometryFilter.cpp
        src/filters/vision/VisionFilter.cpp
        src/filters/vision/robot/RobotFilter.cpp
        src/filters/vision/ball/BallFilter.cpp
        src/filters/vision/PosVelFilter2D.cpp
        src/filters/vision/robot/RobotOrientationFilter.cpp
        src/filters/vision/PosVelFilter1D.cpp
        src/filters/vision/robot/RobotObservation.cpp
        src/filters/vision/robot/RobotPos.cpp
        src/filters/vision/robot/FilteredRobot.cpp
        src/filters/vision/DetectionFrame.cpp
        src/filters/vision/ball/BallObservation.cpp
        include/observer/filters/vision/KalmanFilter.h
        src/filters/vision/ball/GroundBallExtendedKalmanFilter.cpp
        src/filters/vision/ball/FilteredBall.cpp
        src/filters/vision/ball/BallAssigner.cpp
        src/data/RobotParameters.cpp
        src/filters/vision/Camera.cpp
        src/filters/vision/CameraMap.cpp
        src/filters/vision/RobotFeedbackFilter.cpp
        )

target_link_libraries(observer
        PUBLIC roboteam_networking
        PRIVATE roboteam_utils
        PUBLIC Eigen3::Eigen
        )
target_include_directories(observer
        PRIVATE include/observer
        PUBLIC include
        )
target_compile_options(observer PRIVATE "${COMPILER_FLAGS}")

# -- TESTS --
# // TODO: Get this to work @Rolf
#add_executable(roboteam_observer_tests
#        test/main.cpp
#        test/KalmanFilterTest.cpp
#        )
#
#target_include_directories(roboteam_observer_tests
#        PRIVATE include
#        )
#
#target_link_libraries(roboteam_observer_tests
#        PRIVATE observer
#        PRIVATE GTest::gtest
#        PRIVATE GTest::gtest_main
#        )
#target_compile_options(roboteam_observer_tests PRIVATE "${COMPILER_FLAGS}")
#
#add_test(roboteam_observer_tests roboteam_observer_tests)