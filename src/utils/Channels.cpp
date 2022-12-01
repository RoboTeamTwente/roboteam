
#include <utils/Channels.hpp>

namespace rtt::net::utils {

const std::map<ChannelType, Channel> CHANNELS = {{SETTINGS_CHANNEL, {"settings", "127.0.0.1", "5564"}},

                                                 {AI_YELLOW_CHANNEL, {"yellow_ai", "127.0.0.1", "16972"}},
                                                 {AI_BLUE_CHANNEL, {"blue_ai", "127.0.0.1", "16973"}},

                                                 {INTERFACE_TO_AI_CHANNEL, {"ai_to_interface", "127.0.0.1", "16970"}},
                                                 {AI_TO_INTERFACE_CHANNEL, {"interface_to_ai", "127.0.0.1", "16971"}},

                                                 {ROBOT_COMMANDS_BLUE_CHANNEL, {"robot_commands", "127.0.0.1", "5559"}},
                                                 {ROBOT_COMMANDS_YELLOW_CHANNEL, {"robot_commands", "127.0.0.1", "5560"}},

                                                 {ROBOT_FEEDBACK_CHANNEL, {"robot_feedback", "127.0.0.1", "5561"}},

                                                 {WORLD_CHANNEL, {"world", "127.0.0.1", "5558"}},

                                                 {SIMULATION_CONFIGURATION_CHANNEL, {"simulation_configuration", "127.0.0.1", "5562"}}};

}  // namespace rtt::net::utils