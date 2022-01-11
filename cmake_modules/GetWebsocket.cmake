include(FetchContent)

FetchContent_Declare(proj_ixwebsocket
        GIT_REPOSITORY https://github.com/machinezone/IXWebSocket
        GIT_TAG master
        )

FetchContent_MakeAvailable(proj_ixwebsocket)