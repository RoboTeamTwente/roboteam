if (APPLE)
    if (EXISTS /usr/local/opt/qt@5)
        list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/qt@5")
    elseif(EXISTS /usr/local/opt/qt5)
        list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/qt5")
    else()
        message(FATAL_ERROR "Could not find Qt5. Please install with `brew install qt@5`.")
    endif()
elseif(UNIX AND NOT APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/Qt5")
else()
    message(FATAL_ERROR "Can't auto-locate Qt5 for this platform")
endif()

# AUTOMOC is verified to work with the following:
# - libprotoc 3.19.4
# - macOS 12.2.1
# - qt 5.15.2

# On some operating systems, AUTOMOC might produce errors related to protobuf files.
# In such a case, header files to be MOC-ed must be manually specified
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC OFF)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_INCLUDE_CURRENT_DIR OFF) # Find includes in corresponding build directories

find_package(Qt5 COMPONENTS Widgets Charts REQUIRED)
