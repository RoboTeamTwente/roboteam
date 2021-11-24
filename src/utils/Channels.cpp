
#include <utils/Channels.hpp>

namespace rtt::net::utils {

const std::map<ChannelType, Channel> CHANNELS = {{WORLD_CHANNEL, {"world", "127.0.0.1", "5558"}},
                                                 {ROBOT_COMMANDS_CHANNEL, {"robot_commands", "127.0.0.1", "5559"}},
                                                 {ROBOT_FEEDBACK_CHANNEL, {"robot_feedback", "127.0.0.1", "5561"}},
                                                 {SETTINGS_CHANNEL, {"settings", "127.0.0.1", "5564"}}};

}  // namespace rtt::net::utils