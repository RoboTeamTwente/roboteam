protobuf_generate_cpp(NET_PROTO_SRCS NET_PROTO_HDRS
    "messages_robocup_ssl_detection.proto"
    "messages_robocup_ssl_game_controller_common.proto"
    "messages_robocup_ssl_game_controller_geometry.proto"
    "messages_robocup_ssl_game_event.proto"
    "messages_robocup_ssl_geometry.proto"
    "messages_robocup_ssl_referee.proto"
    "messages_robocup_ssl_wrapper.proto"
    
    # Messages from ssl-simulation-control https://github.com/RoboCup-SSL/ssl-simulation-protocol/
    "ssl_gc_common.proto"
    "ssl_simulation_config.proto"
    "ssl_simulation_control.proto" 
    "ssl_simulation_error.proto"
    "ssl_simulation_robot_control.proto"
    "ssl_simulation_robot_feedback.proto" 
    "ssl_vision_geometry.proto"

    "RobotCommands.proto"
    "RobotFeedback.proto"
    "State.proto"
    "Vector2f.proto"
    "World.proto"
    "WorldBall.proto"
    "WorldRobot.proto"
    "GameSettings.proto"
    "RobotParameters.proto"
    "RobotProcessedFeedback.proto"
    "GUI.proto"
)

add_library(roboteam_networking_proto STATIC "${NET_PROTO_SRCS}" "${NET_PROTO_HDRS}")
target_link_libraries(roboteam_networking_proto PUBLIC "${PROTOBUF_LIBRARIES}")
target_include_directories(roboteam_networking_proto PUBLIC "${PROTOBUF_INCLUDE_DIRS}" "${CMAKE_CURRENT_BINARY_DIR}/..")