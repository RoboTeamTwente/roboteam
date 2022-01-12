include(FetchContent)

FetchContent_Declare(proj_sdl2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL
        GIT_TAG a29d3acc9e86eea5712c2b08154c0cc805acd044
        )

FetchContent_MakeAvailable(proj_sdl2)

set_target_properties(SDL2 PROPERTIES UNITY_BUILD FALSE)