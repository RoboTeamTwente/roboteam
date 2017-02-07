cmake_minimum_required(VERSION 2.8.3)
project(roboteam_utils)

# The code to generate protobuf files is mostly from here:
# http://answers.ros.org/question/192882/using-catkin-to-generate-and-use-protobuf-messages/
# See dustingooding's answer.
# Get all the cpp files to compile.

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(catkin REQUIRED COMPONENTS roboteam_msgs rospy)

find_package(Protobuf REQUIRED)

set(CMAKE_CXX_FLAGS "\
--std=c++14 ${CMAKE_CXX_FLAGS} \
-Wall \
-Wno-unused-function \
-Wsign-compare \
-g \
")

set(proto_dir ${PROJECT_SOURCE_DIR}/proto)
set(proto_files
    ${proto_dir}/grSim_Replacement.proto
    ${proto_dir}/grSim_Commands.proto
    ${proto_dir}/grSim_Packet.proto
    ${proto_dir}/messages_robocup_ssl_detection.proto
    ${proto_dir}/messages_robocup_ssl_geometry.proto
    ${proto_dir}/messages_robocup_ssl_geometry_legacy.proto
    ${proto_dir}/messages_robocup_ssl_referee.proto
    ${proto_dir}/messages_robocup_ssl_wrapper.proto
    ${proto_dir}/messages_robocup_ssl_wrapper_legacy.proto
)

# Set up destination directories
catkin_destinations()
set(proto_gen_cpp_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
set(proto_gen_python_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
file(MAKE_DIRECTORY ${proto_gen_cpp_dir})
file(MAKE_DIRECTORY ${proto_gen_python_dir})
set(protogen_include_dirs ${proto_gen_cpp_dir}/../ ${proto_gen_python_dir})

# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include ${protogen_include_dirs}
  LIBRARIES roboteam_utils ${PROJECT_NAME}_proto
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    ${PROTOBUF_INCLUDE_DIR}
    ${PROTOBUF_INCLUDE_DIR}/../../
)

# Create lists of files to be generated.
set(proto_gen_cpp_files "")
set(proto_gen_python_files "")
foreach(proto_file ${proto_files})
    get_filename_component(proto_name ${proto_file} NAME_WE)
    list(APPEND proto_gen_cpp_files ${proto_gen_cpp_dir}/${proto_name}.pb.h ${proto_gen_cpp_dir}/${proto_name}.pb.cc)
    list(APPEND proto_gen_python_files ${proto_gen_python_dir}/${proto_name}_pb2.py)
endforeach(proto_file ${proto_files})

# Run protoc and generate language-specific headers.
add_custom_command(
    OUTPUT ${proto_gen_cpp_files} ${proto_gen_python_files}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} --proto_path=${proto_dir} --cpp_out=${proto_gen_cpp_dir} --python_out=${proto_gen_python_dir} ${proto_files}
    DEPENDS ${PROTOBUF_PROTOC_EXECUTABLE} ${proto_files}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

set_source_files_properties(${proto_gen_cpp_files} PROPERTIES GENERATED TRUE)
set_source_files_properties(${proto_gen_python_files} PROPERTIES GENERATED TRUE)

# Make the python message folder a python package.
file(APPEND ${proto_gen_python_dir}/__init__.py "")

add_custom_target(${PROJECT_NAME}_generate_headers
    DEPENDS ${proto_gen_cpp_files} ${proto_gen_python_files}
)

# Create proto library for linking.
add_library(${PROJECT_NAME}_proto ${proto_gen_cpp_files})
target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARY})
add_dependencies(${PROJECT_NAME}_proto ${PROJECT_NAME}_generate_headers)

install(TARGETS ${PROJECT_NAME}_proto
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${proto_gen_cpp_dir}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ${proto_gen_python_dir}/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)

add_executable(constants_generator src/ConstantsGenerator.cpp)
target_link_libraries(constants_generator ${catkin_LIBRARIES})

add_custom_command(
	OUTPUT ${PROJECT_SOURCE_DIR}/src/constants.cpp
	COMMAND ${PROJECT_SOURCE_DIR}/generate_constants.sh
	DEPENDS constants_generator ${PROJECT_SOURCE_DIR}/String\ Constants
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	COMMENT "Generating constants file"
)
add_custom_target(generate_constants_t
	ALL
	DEPENDS ${PROJECT_SOURCE_DIR}/src/constants.cpp
)

add_library(roboteam_utils src/Vector2.cpp src/Position.cpp src/constants.cpp)
add_dependencies(roboteam_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_constants_t)
target_link_libraries(roboteam_utils ${catkin_LIBRARIES} ${PROJECT_NAME}_proto)




# Protobuf messages.
# add_library(roboteam_utils_proto ${PROTO_CPP} ${PROTO_H} ${PROTO_PY})
# add_dependencies(roboteam_utils_proto ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(roboteam_utils_proto ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} ${PROTOBUF_LIBRARY})

catkin_add_gtest(utils_test
    test/main.cpp
    test/Vector2Test.cpp
    test/PositionTest.cpp
    test/GraphTest.cpp
    src/Vector2.cpp
    src/Position.cpp
)
target_link_libraries(utils_test ${catkin_LIBRARIES})
