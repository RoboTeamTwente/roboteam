cmake_minimum_required(VERSION 3.10)
project(rtt)

set(CMAKE_CXX_STANDARD 20) # -std=c++17
set(CMAKE_CXX_STANDARD_REQUIRED ON) # -std=c++17 required
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # -fPic

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")

# external libraries
# stx
set(STX_BUILD_SHARED 1)
add_subdirectory(thirdparty/stx)

# sdl2, since currently broken in ubuntu libs
add_subdirectory(thirdparty/sdl2)

# internal projects
add_subdirectory(roboteam_utils)
add_subdirectory(roboteam_ai)
add_subdirectory(roboteam_robothub)
add_subdirectory(roboteam_world)
