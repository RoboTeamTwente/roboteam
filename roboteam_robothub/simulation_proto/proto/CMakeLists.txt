# Generate the protobuf files
protobuf_generate_cpp(SIM_PROTO_SRCS SIM_PROTO_HDRS
        "ssl_simulation_config.proto"
        "ssl_simulation_robot_control.proto"
        "ssl_simulation_control.proto"
        "ssl_gc_common.proto"
        "ssl_simulation_error.proto"
        "ssl_simulation_robot_feedback.proto"
        "ssl_vision_geometry.proto"
)

# Create the target that contains the necessary proto files
add_library(simulation_manager_proto STATIC "${SIM_PROTO_SRCS}")
target_link_libraries(simulation_manager_proto PUBLIC "${PROTOBUF_LIBRARIES}")
target_include_directories(simulation_manager_proto PUBLIC "${PROTOBUF_INCLUDE_DIRS}" "${CMAKE_CURRENT_BINARY_DIR}/..")
