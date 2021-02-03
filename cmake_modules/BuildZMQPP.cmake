include(ExternalProject)
set(CMAKE_INSTALL_PREFIX )
ExternalProject_Add(project_zmqpp
        GIT_REPOSITORY https://github.com/zeromq/zmqpp.git
        GIT_TAG 85ae96020f2376c53d2176e04e88e8e51021b748
        CMAKE_ARGS
        -DZMQ_BUILD_DRAFT_API=ON
        -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/libzmqpp
        -DZEROMQ_LIB_DIR=${CMAKE_CURRENT_BINARY_DIR}/libzmq/lib
        -DZEROMQ_INCLUDE_DIR=${CMAKE_CURRENT_BINARY_DIR}/libzmq/include
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/libzmqpp
        DEPENDS lib::zmq
        )


#hide raw library
set_target_properties(project_zmqpp PROPERTIES EXCLUDE_FROM_ALL true)
add_library(lib::zmqpp STATIC IMPORTED)
add_dependencies(lib::zmqpp project_zmqpp lib::zmq)
set_target_properties(lib::zmqpp PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/libzmqpp/lib/libzmqpp-static.a)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/libzmqpp/include)
target_link_libraries(lib::zmqpp
        INTERFACE lib::zmq)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libzmqpp/include) #make the include directory so cmake doesn't whine about it not existing
target_include_directories(lib::zmqpp
        INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/libzmqpp/include)