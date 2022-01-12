protobuf_generate_cpp(NET_PROTO_SRCS NET_PROTO_HDRS
    "messages_robocup_ssl_detection.proto"
    "messages_robocup_ssl_game_controller_common.proto"
    "messages_robocup_ssl_game_controller_geometry.proto"
    "messages_robocup_ssl_game_event.proto"
    "messages_robocup_ssl_geometry.proto"
    "messages_robocup_ssl_geometry_legacy.proto"
    "messages_robocup_ssl_referee.proto"
    "messages_robocup_ssl_wrapper.proto"
    "messages_robocup_ssl_wrapper_legacy.proto"
    "RobotCommand.proto"
    "RobotData.proto"
    "RobotFeedback.proto"
    "State.proto"
    "Vector2f.proto"
    "World.proto"
    "WorldBall.proto"
    "WorldRobot.proto"
    "Setting.proto"
    "RobotParameters.proto"
    "AICommand.proto"
)

add_library(roboteam_networking_proto STATIC "${NET_PROTO_SRCS}" "${NET_PROTO_HDRS}")
target_link_libraries(roboteam_networking_proto PUBLIC "${PROTOBUF_LIBRARIES}")
target_include_directories(roboteam_networking_proto PUBLIC "${PROTOBUF_INCLUDE_DIRS}" "${CMAKE_CURRENT_BINARY_DIR}/..")