#SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake_modules") # TODO: fix

#include(ccache)
#include(cotire)

# find the protobuf compiler and libraries
find_package(Protobuf 3.9.1 REQUIRED)
find_package(zeroMQ REQUIRED)
find_package(zmqpp 4.1.2 REQUIRED)

set(PROTO_BASE_PATH proto/proto)
message(${PROTO_BASE_PATH})

# Generate the .h and .cxx files
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
        ${PROTO_BASE_PATH}/Vector2f.proto
        ${PROTO_BASE_PATH}/Referee.proto
        ${PROTO_BASE_PATH}/RobotCommand.proto
        ${PROTO_BASE_PATH}/RobotFeedback.proto
        ${PROTO_BASE_PATH}/World.proto
        ${PROTO_BASE_PATH}/StringEntry.proto
        ${PROTO_BASE_PATH}/Setting.proto

        ${PROTO_BASE_PATH}/FieldCircularArc.proto
        ${PROTO_BASE_PATH}/FieldLineSegment.proto
        ${PROTO_BASE_PATH}/WorldBall.proto
        ${PROTO_BASE_PATH}/WorldRobot.proto

        #field
        ${PROTO_BASE_PATH}/GeometryData.proto
        ${PROTO_BASE_PATH}/GeometryCameraCalibration.proto
        ${PROTO_BASE_PATH}/GeometryFieldSize.proto

        #demo
        ${PROTO_BASE_PATH}/DemoRobot.proto

        #grsim
        ${PROTO_BASE_PATH}/grSim_Commands.proto
        ${PROTO_BASE_PATH}/grSim_Packet.proto
        ${PROTO_BASE_PATH}/grSim_Replacement.proto

        #mimir
        ${PROTO_BASE_PATH}/mimir_robotcommand.proto

        # ssl input packets
        ${PROTO_BASE_PATH}/messages_robocup_ssl_detection.proto
        ${PROTO_BASE_PATH}/messages_robocup_ssl_game_controller_common.proto
        ${PROTO_BASE_PATH}/messages_robocup_ssl_game_event.proto
        ${PROTO_BASE_PATH}/messages_robocup_ssl_geometry.proto
        ${PROTO_BASE_PATH}/messages_robocup_ssl_geometry_legacy.proto
        ${PROTO_BASE_PATH}/messages_robocup_ssl_referee.proto
        ${PROTO_BASE_PATH}/messages_robocup_ssl_wrapper.proto
        ${PROTO_BASE_PATH}/messages_robocup_ssl_wrapper_legacy.proto
        )

add_library(networking STATIC
        src/Channel.cpp
        src/Channels.cpp
        ${PROTO_SRCS}
        ${PROTO_HDRS}
        )

target_include_directories(networking
        PUBLIC ./include
        )

target_link_libraries(networking
        PUBLIC ${PROTOBUF_LIBRARIES}
        # since Publishers/subscribers are templates these 2 libraries need to be PUBLIC
        PUBLIC ${ZeroMQ_LIBRARIES}
        PUBLIC ${ZMQPP_LIBRARIES}
        )