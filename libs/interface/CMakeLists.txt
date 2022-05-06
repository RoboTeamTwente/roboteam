find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC OFF)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_INCLUDE_CURRENT_DIR OFF) #Find includes in corresponding build directories

add_library(roboteam_interface_lib
        src/InterfaceFieldScene.cpp
        src/InterfaceFieldRenderer.cpp
        src/InterfaceRobotPathsRenderer.cpp
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
        src/InterfaceWidgetStopButton.cpp
        src/InterfaceWidgetRobotOverview.cpp
        src/InterfaceWidgetRobotDisplay.cpp
        src/InterfaceSyncedCheckableButton.cpp
        src/InterfaceWidgetDebugDisplay.cpp

        include/MainWindow.h
        include/InterfaceSyncedRadio.h
        include/InterfaceSyncedDropdown.h
        include/InterfaceSyncedText.h
        include/InterfaceSyncedCheckbox.h
        include/InterfaceSyncedSlider.h
        include/InterfaceFieldScene.h
        include/InterfaceFieldView.h
        include/InterfaceWidgetStopButton.h
        include/InterfaceWidgetRobotOverview.h
        include/InterfaceWidgetRobotDisplay.h
        include/InterfaceSyncedCheckableButton.h
        include/InterfaceWidgetDebugDisplay.h
        include/InterfaceControllerClient.h
        )

target_link_libraries(roboteam_interface_lib
        PUBLIC Qt5::Widgets
        PUBLIC roboteam_interface_utils_lib)

target_include_directories(roboteam_interface_lib
        PUBLIC include/)


target_compile_options(roboteam_interface_lib PRIVATE "${COMPILER_FLAGS}")