add_library(observer STATIC
        src/filters/WorldFilter.cpp
        src/filters/RobotFilter.cpp
        src/filters/BallFilter.cpp
        src/filters/CameraFilter.cpp
        src/filters/Scaling.h
        src/Observer.cpp
        src/parameters/RobotParameterDatabase.cpp
        src/filters/geometry/GeometryFilter.cpp
        src/filters/referee/RefereeFilter.cpp src/filters/referee/RefereeFilter.h)
target_link_libraries(observer
        PUBLIC data
        PUBLIC roboteam_proto
        PRIVATE roboteam_utils
        PRIVATE Eigen3::Eigen
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
