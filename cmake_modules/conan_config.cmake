find_program(CONAN_EXE "conan")
if(CONAN_EXE STREQUAL CONAN_EXE-NOTFOUND)
  message(FATAL_ERROR "Unable to find conan")
else()
  message(STATUS "Found conan. Installing dependencies.")
  if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(
      STATUS
        "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(
      DOWNLOAD
      "https://raw.githubusercontent.com/conan-io/cmake-conan/0.18.1/conan.cmake"
      "${CMAKE_BINARY_DIR}/conan.cmake"
      EXPECTED_HASH
        SHA256=5cdb3042632da3efff558924eecefd580a0e786863a857ca097c3d1d43df5dcd
      TLS_VERIFY ON)
  endif()
  include(${CMAKE_BINARY_DIR}/conan.cmake)
  conan_cmake_configure(GENERATORS cmake_find_package)
  conan_cmake_autodetect(settings)
  conan_cmake_install(
    REFERENCE
    "${${CMAKE_PROJECT_NAME}_CONANFILE}"
    BUILD
    missing
    outdated
    REMOTE
    conancenter
    flatexdegiro
    UPDATE
    SETTINGS
    ${settings})
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
  list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
endif()
