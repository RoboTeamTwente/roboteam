include(FetchContent)

FetchContent_Declare(IXWebSocket
        GIT_REPOSITORY https://github.com/machinezone/IXWebSocket.git
        GIT_TAG v11.4.5
        GIT_SHALLOW TRUE
 )

FetchContent_MakeAvailable(IXWebSocket)