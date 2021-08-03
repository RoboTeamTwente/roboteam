include(FetchContent)

FetchContent_Declare(proj_libzmqpp
        GIT_REPOSITORY https://github.com/zeromq/zmqpp.git
        GIT_TAG 85ae96020f2376c53d2176e04e88e8e51021b748
        )

set(ZMQ_BUILD_DRAFT_API true)
set(ZMQPP_LIBZMQ_CMAKE false)

FetchContent_Declare(proj_sdl2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL
        GIT_TAG main
        )

FetchContent_Declare(proj_stx
        GIT_REPOSITORY https://github.com/lamarrr/STX
        GIT_TAG master
        )

FetchContent_Declare(proj_ixwebsocket
        GIT_REPOSITORY https://github.com/machinezone/IXWebSocket
        GIT_TAG master
        )

#set(STX_BUILD_SHARED TRUE)

FetchContent_MakeAvailable(proj_libzmqpp proj_sdl2 proj_stx proj_ixwebsocket)

target_include_directories(zmqpp INTERFACE ${proj_libzmqpp_SOURCE_DIR}/src)

set_target_properties(SDL2 PROPERTIES UNITY_BUILD FALSE)