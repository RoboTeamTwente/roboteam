include(FetchContent)

FetchContent_Declare(proj_sdl2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL
        GIT_TAG d948e6c3c592db7bfdcf14d4e869ff8db238e50a
        )

FetchContent_MakeAvailable(proj_sdl2)

set_target_properties(SDL2 PROPERTIES UNITY_BUILD FALSE)