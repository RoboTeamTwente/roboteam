
find_path(
        ZMQPP_INCLUDE_DIRS
        NAMES zmqpp/zmqpp.hpp
        HINTS ${ZMQPP_INCLUDE_DIRS}
)

find_library(
        ZMQPP_LIBRARIES
        NAMES zmqpp
        HINTS ${ZMQPP_LIBRARY_DIRS}
)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
        ZMQPP
        DEFAULT_MSG
        ZMQPP_LIBRARIES
        ZMQPP_INCLUDE_DIRS
)

if(ZMQPP_FOUND)
    mark_as_advanced(ZMQPP_LIBRARIES ZMQPP_INCLUDE_DIRS)
endif()