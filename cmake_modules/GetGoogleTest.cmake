include(FetchContent)

FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        e2239ee6043f73722e7aa812a459f54a28552929 # v1.11.0
)

FetchContent_MakeAvailable(googletest)

#for MacOS X or iOS, watchOS, tvOS(since 3.10.3)
if (APPLE)
    set(GTEST_LIB
            /usr/local/lib/libgtest.a
            /usr/local/lib/libgtest_main.a
            /usr/local/lib/libgmock.a
            /usr/local/lib/libgmock_main.a
            )
else (NOT APPLE)
    set(GTEST_LIB
            PUBLIC gtest
            PUBLIC gmock)
endif ()
