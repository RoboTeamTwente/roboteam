protobuf_generate_cpp(NET_PROTO_SRCS NET_PROTO_HDRS
        "RobotCommands.proto"
        "RobotFeedback.proto"
        "Settings.proto"
        "FieldGeometry.proto"
        "World.proto"
    )

add_library(roboteam_networking_proto STATIC "${NET_PROTO_SRCS}" "${NET_PROTO_HDRS}")
target_link_libraries(roboteam_networking_proto PUBLIC "${PROTOBUF_LIBRARIES}")
target_include_directories(roboteam_networking_proto PUBLIC "${PROTOBUF_INCLUDE_DIRS}" "${CMAKE_CURRENT_BINARY_DIR}/..")