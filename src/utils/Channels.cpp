
#include <utils/Channels.hpp>

namespace rtt::net::utils {

const std::map<ChannelType, Channel> CHANNELS = {
    {SETTINGS_CHANNEL, {"settings", "127.0.0.1", "5564"}},

    {ROBOT_COMMANDS_BLUE_CHANNEL, {"robot_commands", "127.0.0.1", "5559"}},
    {ROBOT_COMMANDS_YELLOW_CHANNEL, {"robot_commands", "127.0.0.1", "5560"}},

    {ROBOT_FEEDBACK_CHANNEL, {"robot_feedback", "127.0.0.1", "5561"}},

    {WORLD_CHANNEL, {"world", "127.0.0.1", "5558"}}
};

}  // namespace rtt::net::utils