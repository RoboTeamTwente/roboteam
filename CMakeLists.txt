cmake_minimum_required(VERSION 3.16)
project(roboteam_world)

find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Network)


add_executable(roboteam_observer
        src/roboteam_observer/main.cpp
        src/roboteam_observer/Handler.cpp
        )

target_link_libraries(roboteam_observer
        PRIVATE observer_lib
        PRIVATE observer_net_lib
        PRIVATE roboteam_proto
        PRIVATE roboteam_utils
        PRIVATE Qt5::Network
        )

target_include_directories(roboteam_observer
        PRIVATE include/roboteam_observer
        INTERFACE include
        )

# -- OBSERVER LIB --

add_library(observer_lib STATIC
        src/observer/filters/WorldFilter.cpp
        src/observer/filters/RobotFilter.cpp
        src/observer/filters/BallFilter.cpp
        src/observer/filters/CameraFilter.cpp
        src/observer/filters/Scaling.h
        src/observer/Observer.cpp
        src/observer/parameters/RobotParameterDatabase.cpp
        src/observer/filters/geometry/GeometryFilter.cpp
        src/observer/filters/referee/RefereeFilter.cpp
        src/observer/filters/RobotFeedbackFilter.cpp)

target_link_libraries(observer_lib
        PUBLIC observer_data_lib
        PUBLIC roboteam_proto
        PRIVATE roboteam_utils
        PRIVATE Eigen3::Eigen
        )

target_include_directories(observer_lib
        PRIVATE include/observer
        PUBLIC include
        )

set_target_properties(observer_lib PROPERTIES UNITY_BUILD FALSE)

# -- OBSERVER_TESTS --

add_executable(observer_lib_tests
        test/observer/main.cpp
        test/observer/KalmanFilterTest.cpp
        )

target_link_libraries(observer_lib_tests
        PRIVATE observer
        PRIVATE gtest
        )

# -- NET --

add_library(observer_net_lib STATIC
        src/net/netraw.cpp
        src/net/robocup_ssl_client.cpp
        )

target_include_directories(observer_net_lib
        PRIVATE include/net
        INTERFACE include
        )

target_link_libraries(observer_net_lib
        PUBLIC roboteam_proto
        )

# -- DATA --

add_library(observer_data_lib STATIC
        src/data/RobotParameters.cpp
        )

target_include_directories(observer_data_lib
        PRIVATE include/data
        INTERFACE include
        )

target_link_libraries(observer_data_lib
        PUBLIC roboteam_proto)
