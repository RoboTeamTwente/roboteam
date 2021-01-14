find_package(Sodium)
include(ExternalProject)

ExternalProject_Add(project_zeromq
        BUILD_IN_SOURCE true
        URL https://github.com/zeromq/libzmq/releases/download/v4.3.3/zeromq-4.3.3.tar.gz
        PREFIX "${CMAKE_CURRENT_BINARY_DIR}/libzmq"
        CMAKE_ARGS
        -DENABLE_DRAFTS=ON
        -DBUILD_TESTING=OFF
        -DZMQ_BUILD_FRAMEWORK=OFF
        CONFIGURE_COMMAND ./configure --with-libsodium --without-docs --prefix=${CMAKE_CURRENT_BINARY_DIR}/libzmq
)

add_library(lib::zmq STATIC IMPORTED)
add_dependencies(lib::zmq project_zeromq)
set_target_properties(lib::zmq PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/libzmq/lib/libzmq.a)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/libzmq/include)
target_link_libraries(lib::zmq
        INTERFACE ${sodium_LIBRARIES})
#target_include_directories(lib::zmq
#        INTERFACE ${CMAKE_CURRENT_BINARY_DIR})