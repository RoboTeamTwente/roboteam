cmake_minimum_required(VERSION 3.10)
project(rtt)
message("using compiler: ${CMAKE_CXX_COMPILER}")

set(CMAKE_CXX_STANDARD 20) # -std=c++17
set(CMAKE_CXX_STANDARD_REQUIRED ON) # -std=c++17 required
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # -fPic

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

include(cotire)
include(ccache)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

include(cmake_modules/BuildSodium.cmake)
include(cmake_modules/BuildZeroMQ.cmake)

# external libraries
# stx
set(STX_BUILD_SHARED 1)
add_subdirectory(thirdparty/stx)
add_subdirectory(thirdparty/sdl2)
# IXWebsocket
add_subdirectory(thirdparty/IXWebSocket)
include(cmake_modules/BuildZMQPP.cmake)



# internal projects
add_subdirectory(roboteam_proto)
add_subdirectory(roboteam_utils)
add_subdirectory(roboteam_ai)
add_subdirectory(roboteam_robothub)
add_subdirectory(roboteam_world)
add_subdirectory(roboteam_central_server)
