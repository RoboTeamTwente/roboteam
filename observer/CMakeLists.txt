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
        src/filters/vision/ball/FilteredBall.cpp src/filters/vision/ball/FilteredBall.h)
target_link_libraries(observer
        PUBLIC data
        PUBLIC roboteam_proto
        PRIVATE roboteam_utils
        PUBLIC Eigen3::Eigen
        )
target_include_directories(observer
        PRIVATE include/observer
        PUBLIC include
        )
target_compile_options(observer
        PUBLIC
        -O3
        -std=c++11
        )


add_executable(observer_tests
        test/main.cpp
        test/KalmanFilterTest.cpp
        )

target_link_libraries(observer_tests
        PRIVATE observer
        PRIVATE gtest
        )
target_compile_options(observer_tests
        PRIVATE
        -O0
        -g
        )
