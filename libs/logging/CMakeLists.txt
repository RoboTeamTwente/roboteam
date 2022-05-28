add_library(roboteam_logging
        src/LogFileReader.cpp
        src/LogFileWriter.cpp
        )

target_include_directories(roboteam_logging
        PRIVATE include/roboteam_logging
        PUBLIC include/
        )

target_link_libraries(roboteam_logging
        PUBLIC roboteam_networking  #TODO maybe separate proto/networking dependencies more clearly
                                    #this is not necessary here
        )
target_compile_options(roboteam_logging PRIVATE "${COMPILER_FLAGS}")

add_executable(roboteam_logging_tests
        test/LoggingTest.cpp)

target_link_libraries(roboteam_logging_tests
        PRIVATE roboteam_logging
        PRIVATE GTest::gtest
        )

target_compile_options(roboteam_logging_tests PRIVATE "${COMPILER_FLAGS}")