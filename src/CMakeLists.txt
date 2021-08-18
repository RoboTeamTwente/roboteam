set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_library(roboteam_interface_lib
        InterfaceDeclaration.cpp
        InterfaceDeclarations.cpp
        InterfaceSettings.cpp
        InterfaceValue.cpp)

target_include_directories(roboteam_interface_lib
        PRIVATE ../include/roboteam_interface
        INTERFACE ../include)

target_link_libraries(roboteam_interface_lib
        PUBLIC Qt5::Widgets
        PUBLIC roboteam_proto
        PUBLIC nlohmann_json::nlohmann_json
        PRIVATE roboteam_utils)