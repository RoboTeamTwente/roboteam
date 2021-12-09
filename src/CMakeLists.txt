#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)
#set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

#set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_wrap_cpp(IFACE_SRC ../include/roboteam_interface/MainWindow.h ../include/roboteam_interface/InterfaceFieldWidget.h ../include/roboteam_interface/InterfaceSyncedCheckbox.h ../include/roboteam_interface/InterfaceSyncedSlider.h ../include/roboteam_interface/InterfaceSyncedText.h ../include/roboteam_interface/InterfaceSyncedDropdown.h ../include/roboteam_interface/InterfaceSyncedRadio.h)

add_library(roboteam_interface_lib
        InterfaceDeclaration.cpp
        InterfaceDeclarations.cpp
        InterfaceSettings.cpp
        InterfaceValue.cpp
        InterfaceController.cpp
        MainWindow.cpp
        InterfaceDeclarationsRenderer.cpp
        InterfaceFieldWidget.cpp
        InterfaceSyncedCheckbox.cpp
        InterfaceSyncedSlider.cpp
        InterfaceSyncedDropdown.cpp
        InterfaceSyncedRadio.cpp
        InterfaceSyncedText.cpp
        ${IFACE_SRC}
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