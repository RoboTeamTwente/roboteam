#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)
#set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
qt_wrap_cpp(IFACE_MOC ../include/roboteam_interface/InterfaceFieldScene.h
        ../include/roboteam_interface/InterfaceSyncedCheckbox.h
        ../include/roboteam_interface/InterfaceSyncedDropdown.h
        ../include/roboteam_interface/InterfaceSyncedRadio.h
        ../include/roboteam_interface/InterfaceSyncedSlider.h
        ../include/roboteam_interface/InterfaceSyncedText.h
        ../include/roboteam_interface/MainWindow.h)

#set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(roboteam_interface_lib
        InterfaceDeclaration.cpp
        InterfaceDeclarations.cpp
        InterfaceSettings.cpp
        InterfaceValue.cpp
        InterfaceController.cpp
        InterfaceFieldScene.cpp
        InterfaceFieldRenderer.cpp
        MainWindow.cpp
        InterfaceSyncedCheckbox.cpp
        InterfaceSyncedSlider.cpp
        InterfaceSyncedDropdown.cpp
        InterfaceSyncedRadio.cpp
        InterfaceSyncedText.cpp
        InterfaceFieldStateStore.cpp
        InterfaceRobotItem.cpp
        ${IFACE_MOC}
        )


target_include_directories(roboteam_interface_lib
        PRIVATE ../include/roboteam_interface
        PRIVATE ../include/roboteam_interface/widgets
        INTERFACE ../include)

target_link_libraries(roboteam_interface_lib
        PUBLIC Qt5::Widgets
        PUBLIC roboteam_proto
        PUBLIC nlohmann_json::nlohmann_json
        PRIVATE roboteam_utils)