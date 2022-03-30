add_library(interface_utils_lib
        src/InterfaceDeclaration.cpp
        src/InterfaceDeclarations.cpp
        src/InterfaceSettings.cpp
        src/InterfaceValue.cpp)

target_include_directories(interface_utils_lib
        PRIVATE include/roboteam_interface_utils
        INTERFACE include/
        )

target_link_libraries(interface_utils_lib
        PUBLIC roboteam_networking
        PUBLIC nlohmann_json::nlohmann_json
        PUBLIC roboteam_utils)

target_compile_options(interface_utils_lib PRIVATE "${COMPILER_FLAGS}")