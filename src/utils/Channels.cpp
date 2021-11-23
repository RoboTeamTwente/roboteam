
#include <utils/Channels.hpp>

namespace rtt::net::utils {

const std::map<ChannelType, Channel> CHANNELS = {{GEOMETRY_CHANNEL, {"geometry", "127.0.0.1", "5556"}},
                                                 {REFEREE_CHANNEL, {"referee", "127.0.0.1", "5557"}},
                                                 {WORLD_CHANNEL, {"world", "127.0.0.1", "5558"}},
                                                 {ROBOT_COMMANDS_PRIMARY_CHANNEL, {"commands_primary", "127.0.0.1", "5559"}},
                                                 {ROBOT_COMMANDS_SECONDARY_CHANNEL, {"commands_secondary", "127.0.0.1", "5560"}},
                                                 {FEEDBACK_PRIMARY_CHANNEL, {"feedback_primary", "127.0.0.1", "5561"}},
                                                 {FEEDBACK_SECONDARY_CHANNEL, {"feedback_secondary", "127.0.0.1", "5562"}},
                                                 {SETTINGS_PRIMARY_CHANNEL, {"settings_primary", "127.0.0.1", "5563"}},
                                                 {SETTINGS_SECONDARY_CHANNEL, {"settings_secondary", "127.0.0.1", "5564"}}};

}  // namespace rtt::net::utils