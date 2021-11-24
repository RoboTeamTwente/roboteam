add_executable(loopsTest "LoopsTest.cpp")
target_link_libraries(loopsTest PUBLIC networking)

add_executable(sharedChannelTest "SharedChannelsTest.cpp")
target_link_libraries(sharedChannelTest PUBLIC networking)