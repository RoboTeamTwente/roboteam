find_package(Qt5 COMPONENTS Widgets REQUIRED)
qt_wrap_cpp(IFACE_MOC
        include/InterfaceFieldScene.h
        include/InterfaceSyncedCheckbox.h
        include/InterfaceSyncedDropdown.h
        include/InterfaceSyncedRadio.h
        include/InterfaceSyncedSlider.h
        include/InterfaceSyncedText.h
        include/MainWindow.h
        include/InterfaceFieldView.h)

add_library(interface_lib
        src/InterfaceFieldScene.cpp
        src/InterfaceFieldRenderer.cpp
        src/MainWindow.cpp
        src/InterfaceSyncedCheckbox.cpp
        src/InterfaceSyncedSlider.cpp
        src/InterfaceSyncedDropdown.cpp
        src/InterfaceSyncedRadio.cpp
        src/InterfaceSyncedText.cpp
        src/InterfaceFieldStateStore.cpp
        src/InterfaceRobotItem.cpp
        src/InterfaceBallItem.cpp
        src/InterfaceFieldView.cpp
        src/InterfaceControllerClient.cpp
        ${IFACE_MOC})

target_link_libraries(interface_lib
        PUBLIC Qt::Widgets
        PUBLIC interface_utils_lib)

target_include_directories(interface_lib
        PUBLIC include/)