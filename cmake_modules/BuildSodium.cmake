include(ExternalProject)
set(sodium_src_dir   ${CMAKE_CURRENT_BINARY_DIR}/libsodium)
set(sodium_build_dir ${CMAKE_CURRENT_BINARY_DIR}/libsodium)
set(LIBSODIUM_LIB_DIR     ${sodium_build_dir}/lib)
set(LIBSODIUM_INCLUDE_DIR ${sodium_src_dir}/src/libsodium/include)

ExternalProject_Add(project_libsodium
        GIT_REPOSITORY https://github.com/jedisct1/libsodium.git
        GIT_TAG a5e2122177fa88f09b72047030f3a23f5b651c49
        PREFIX     ${CMAKE_CURRENT_BINARY_DIR}/libsodium
        CONFIGURE_COMMAND echo "test echo?!?!??!?!?"
        BUILD_COMMAND
        ${CMAKE_CURRENT_BINARY_DIR}/libsodium/autogen.sh
        ${CMAKE_CURRENT_BINARY_DIR}/libsodium/configure
        INSTALL_COMMAND make -j8 install
        BUILD_BYPRODUCTS ${LIBSODIUM_LIB_DIR}/libsodium.a
        )
add_library(lib::sodium STATIC IMPORTED)
add_dependencies(lib::sodium project_libsodium)
#set_target_properties(lib::sodium PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/libsodium/lib/libsodium.a)
#include_directories(${CMAKE_CURRENT_BINARY_DIR}/libzmq/include)
#target_include_directories(lib::zmq
#        INTERFACE ${CMAKE_CURRENT_BINARY_DIR})