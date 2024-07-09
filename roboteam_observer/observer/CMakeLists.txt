file(GLOB_RECURSE OBSERVER_SOURCES 
        "${CMAKE_CURRENT_SOURCE_DIR}/src/filters/vision/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/filters/vision/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/observer/filters/vision/*.h"
)

add_library(observer STATIC ${OBSERVER_SOURCES})
find_package(dlib REQUIRED)
target_link_libraries(observer
        PUBLIC dlib::dlib
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
#gtest_discover_tests(roboteam_observer_tests)