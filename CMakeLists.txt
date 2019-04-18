cmake_minimum_required(VERSION 2.8.3)
project(roboteam_utils)

# The code to generate protobuf files is mostly from here:
# http://answers.ros.org/question/192882/using-catkin-to-generate-and-use-protobuf-messages/
# See dustingooding's answer.
# Get all the cpp files to compile.

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(ENABLE_DEBUG_GRAPHICS TRUE)

if (ENABLE_DEBUG_GRAPHICS)
    add_definitions(-DRTT_CMAKE_ENABLE_DEBUG_GRAPHICS=1)
else (ENABLE_DEBUG_GRAPHICS)
    add_definitions(-DRTT_CMAKE_ENABLE_DEBUG_GRAPHICS=0)
endif (ENABLE_DEBUG_GRAPHICS)

find_package(catkin REQUIRED COMPONENTS roboteam_msgs rospy roscpp roslib)

find_package(Protobuf REQUIRED)

set(CMAKE_CXX_FLAGS "\
--std=c++14 ${CMAKE_CXX_FLAGS} \
-Wall \
-Wno-unused-function \
-Wsign-compare \
-g \
"
        )

#####################################
## Protobuf conversion & compiling ##
#####################################

set(proto_dir ${PROJECT_SOURCE_DIR}/proto)
set(proto_files
        ${proto_dir}/grSim_Replacement.proto
        ${proto_dir}/grSim_Commands.proto
        ${proto_dir}/grSim_Packet.proto
        ${proto_dir}/messages_robocup_ssl_detection.proto
        ${proto_dir}/messages_robocup_ssl_geometry.proto
        ${proto_dir}/messages_robocup_ssl_geometry_legacy.proto
        ${proto_dir}/messages_robocup_ssl_referee.proto
        ${proto_dir}/messages_robocup_ssl_game_event.proto
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
        INCLUDE_DIRS
        include
        ${protogen_include_dirs}
        LIBRARIES
        roboteam_utils
        ${PROJECT_NAME}_proto
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
foreach (proto_file ${proto_files})
    get_filename_component(proto_name ${proto_file} NAME_WE)
    list(APPEND proto_gen_cpp_files ${proto_gen_cpp_dir}/${proto_name}.pb.h ${proto_gen_cpp_dir}/${proto_name}.pb.cc)
    list(APPEND proto_gen_python_files ${proto_gen_python_dir}/${proto_name}_pb2.py)
endforeach (proto_file ${proto_files})

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

##########################
## Constants generation ##
##########################

set(RTT_CONSTANTS_HEADER_FILE "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/constants.h")
set(RTT_CONSTANTS_SOURCE_FILE "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/roboteam_utils/constants.cpp")
configure_file(include/roboteam_utils/constants.h ${RTT_CONSTANTS_HEADER_FILE} COPYONLY)
configure_file(src/constants.cpp ${RTT_CONSTANTS_SOURCE_FILE} COPYONLY)

###########################
## Regular library stuff ##
###########################

add_library(roboteam_utils
        src/Vector2.cpp
        src/Angle.cpp
        src/Position.cpp
        src/LastWorld.cpp
        src/Draw.cpp
        src/Math.cpp
        src/Cone.cpp
        src/normalize.cpp
        src/Clip.cpp
        src/Optimization.cpp
        src/Arc.cpp
        src/Section.cpp
        ${RTT_CONSTANTS_SOURCE_FILE}
        )
add_dependencies(roboteam_utils
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(roboteam_utils
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_proto
        )

# Protobuf messages.
# add_library(roboteam_utils_proto ${PROTO_CPP} ${PROTO_H} ${PROTO_PY})
# add_dependencies(roboteam_utils_proto ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(roboteam_utils_proto ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} ${PROTOBUF_LIBRARY})

catkin_add_gtest(utils_test
        test/main.cpp
        test/Vector2Test.cpp
        test/PositionTest.cpp
        test/GraphTest.cpp
        test/OptimizationTest.cpp
        test/BiMapTest.cpp
        test/ArcTest.cpp
        #test/normaliseTest.cpp
        )
add_dependencies(utils_test
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(utils_test
        roboteam_utils
        ${catkin_LIBRARIES}
        )
