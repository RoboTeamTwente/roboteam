include(ExternalProject)
set(sodium_src_dir   ${CMAKE_CURRENT_SOURCE_DIR}/libsodium)
set(sodium_build_dir ${CMAKE_CURRENT_BINARY_DIR}/libsodium)
set(LIBSODIUM_LIB_DIR     ${sodium_build_dir}/lib)
set(LIBSODIUM_INCLUDE_DIR ${sodium_src_dir}/src/libsodium/include)

ExternalProject_Add(project_libsodium
        URL https://github.com/jedisct1/libsodium/archive/1.0.18.tar.gz
        PREFIX     ${sodium_build_dir}
        SOURCE_DIR ${sodium_src_dir}
        BINARY_DIR ${sodium_src_dir}
        CONFIGURE_COMMAND ${sodium_src_dir}/autogen.sh -S
        BUILD_COMMAND ${sodium_src_dir}/configure --prefix=${sodium_build_dir}
        INSTALL_COMMAND make -j8 install
        BUILD_BYPRODUCTS ${LIBSODIUM_LIB_DIR}/libsodium.a
        )
add_library(lib::sodium STATIC IMPORTED)
add_dependencies(lib::sodium project_libsodium)
set_target_properties(lib::sodium PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/libsodium/lib/libsodium.a)