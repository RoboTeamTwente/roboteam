protobuf_generate_cpp(NET_PROTO_SRCS NET_PROTO_HDRS
        "RobotCommands.proto"
        "RobotFeedbacks.proto"
        "Settings.proto"
    )

add_library(networking_proto STATIC "${NET_PROTO_SRCS}" "${NET_PROTO_HDRS}")
target_link_libraries(networking_proto PUBLIC "${PROTOBUF_LIBRARIES}")
target_include_directories(networking_proto PUBLIC "${PROTOBUF_INCLUDE_DIRS}" "${CMAKE_CURRENT_BINARY_DIR}/..")