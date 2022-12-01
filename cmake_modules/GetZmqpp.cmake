include(FetchContent)

FetchContent_Declare(proj_libzmqpp
        GIT_REPOSITORY https://github.com/zeromq/zmqpp.git
        GIT_TAG 85ae96020f2376c53d2176e04e88e8e51021b748
        )

set(ZMQ_BUILD_DRAFT_API true)
set(ZMQPP_LIBZMQ_CMAKE false)

FetchContent_MakeAvailable(proj_libzmqpp)

target_include_directories(zmqpp BEFORE INTERFACE ${proj_libzmqpp_SOURCE_DIR}/src)