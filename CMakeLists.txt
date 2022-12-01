cmake_minimum_required(VERSION 3.16)
project(roboteam_networking VERSION 1.0)

set(CMAKE_CXX_STANDARD "17")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")
include(GetDependencies)

# Create make file for all theproto files as target 'roboteam_networking_proto'
find_package(Protobuf 3.9.1 REQUIRED)
add_subdirectory(proto)

add_library(roboteam_networking_utils STATIC
    "src/utils/Channel.cpp"
    "src/utils/Channels.cpp"
    "src/utils/Subscriber.cpp"
    "src/utils/Publisher.cpp"
)
target_include_directories(roboteam_networking_utils PUBLIC "include")
target_link_libraries(roboteam_networking_utils PUBLIC zmqpp)
target_compile_options(roboteam_networking_utils PRIVATE "${COMPILER_FLAGS}")

add_library(roboteam_networking STATIC
    "src/RobotCommandsNetworker.cpp"
    "src/RobotFeedbackNetworker.cpp"
    "src/SettingsNetworker.cpp"
    "src/WorldNetworker.cpp"
    "src/SimulationConfigurationNetworker.cpp"
    "src/AIDataNetworker.cpp"
)

target_include_directories(roboteam_networking PUBLIC "include")
target_link_libraries(roboteam_networking PUBLIC
    roboteam_networking_utils
    roboteam_networking_proto
    roboteam_utils
)
target_compile_options(roboteam_networking PRIVATE "${COMPILER_FLAGS}")

# -- TESTS --

add_executable(roboteam_networking_tests
        tests/LoopsTest.cpp
        tests/SharedChannelsTest.cpp)

target_link_libraries(roboteam_networking_tests
        PRIVATE roboteam_networking
        PRIVATE GTest::gtest
        PRIVATE GTest::gtest_main
)

gtest_discover_tests(roboteam_networking_tests)

add_executable(roboteam_networking_systest
        system_tests/SendRobotCommands.cpp)

target_link_libraries(roboteam_networking_systest
        PRIVATE roboteam_networking)

