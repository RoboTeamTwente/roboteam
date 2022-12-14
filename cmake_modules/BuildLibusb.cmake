include(ExternalProject)

set(libusb_src_dir   ${CMAKE_CURRENT_BINARY_DIR}/libusb/build)
set(libusb_build_dir ${CMAKE_CURRENT_BINARY_DIR}/libusb)
set(LIBUSB_LIB_DIR     ${libusb_build_dir}/lib)
set(LIBUSB_INCLUDE_DIR ${libusb_build_dir}/include)

if (UNIX AND NOT APPLE)
    find_package(udev)
    ExternalProject_Add(project_libusb
            URL https://github.com/libusb/libusb/archive/refs/tags/v1.0.24.tar.gz
            PREFIX     ${libusb_build_dir}
            SOURCE_DIR ${libusb_src_dir}
            BINARY_DIR ${libusb_src_dir}
            CONFIGURE_COMMAND ${libusb_src_dir}/autogen.sh
            BUILD_COMMAND ${libusb_src_dir}/configure --prefix=${libusb_build_dir} --with-libudev=${UDEV_LIBRARY}
            INSTALL_COMMAND make -j3 install
            BUILD_BYPRODUCTS ${LIBUSB_LIB_DIR}/libusb-1.0.a
            )
else()
    # No UDEV on macOS
    ExternalProject_Add(project_libusb
            URL https://github.com/libusb/libusb/archive/refs/tags/v1.0.24.tar.gz
            PREFIX     ${libusb_build_dir}
            SOURCE_DIR ${libusb_src_dir}
            BINARY_DIR ${libusb_src_dir}
            CONFIGURE_COMMAND ${libusb_src_dir}/autogen.sh
            BUILD_COMMAND ${libusb_src_dir}/configure --prefix=${libusb_build_dir}
            INSTALL_COMMAND make -j3 install
            BUILD_BYPRODUCTS ${LIBUSB_LIB_DIR}/libusb-1.0.a
            )
endif()

add_library(lib::usb STATIC IMPORTED)
add_dependencies(lib::usb project_libusb)

set_target_properties(lib::usb PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/libusb/lib/libusb-1.0.a)

target_include_directories(lib::usb
        INTERFACE ${LIBUSB_INCLUDE_DIR})

if (UNIX AND NOT APPLE)
    target_link_libraries(lib::usb
            INTERFACE ${UDEV_LIBRARY})
elseif(APPLE)
    find_library(CF CoreFoundation)
    find_library(IK IOKit)
    target_link_libraries(lib::usb INTERFACE ${CF} ${IK})
endif()


file(MAKE_DIRECTORY ${LIBUSB_INCLUDE_DIR})