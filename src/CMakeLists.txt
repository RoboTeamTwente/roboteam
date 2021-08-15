set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_library(roboteam_interface_lib init.cpp)

target_include_directories(roboteam_interface_lib
        PRIVATE ../include/interface
        INTERFACE ../include)

target_link_libraries(roboteam_interface_lib
        PUBLIC Qt5::Widgets
        PRIVATE utils_networking_lib)