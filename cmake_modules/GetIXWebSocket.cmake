include(FetchContent)

FetchContent_Declare(IXWebSocket
        GIT_REPOSITORY https://github.com/machinezone/IXWebSocket.git
        GIT_TAG master
 )

FetchContent_MakeAvailable(IXWebSocket)