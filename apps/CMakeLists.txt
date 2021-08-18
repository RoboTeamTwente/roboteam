add_executable(roboteam_interface main.cpp)

target_link_libraries(roboteam_interface
        PRIVATE roboteam_interface_lib
        PRIVATE utils_networking_lib)