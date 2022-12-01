add_executable(roboteam_interface main.cpp)

target_link_libraries(roboteam_interface
        PRIVATE roboteam_interface_lib
        PRIVATE roboteam_interface_utils_lib)

target_compile_options(roboteam_interface PRIVATE "${COMPILER_FLAGS}")