add_library(roboteam_interface_utils_lib
        src/InterfaceDeclaration.cpp
        src/InterfaceDeclarations.cpp
        src/InterfaceSettings.cpp
        src/InterfaceValue.cpp)

target_include_directories(roboteam_interface_utils_lib
        PRIVATE include/roboteam_interface_utils
        INTERFACE include/
        )

target_link_libraries(roboteam_interface_utils_lib
        PUBLIC roboteam_networking
        PUBLIC roboteam_utils)

target_compile_options(roboteam_interface_utils_lib PRIVATE "${COMPILER_FLAGS}")