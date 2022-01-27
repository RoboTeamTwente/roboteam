#pragma once
#include <map>
#include <string>
#include <utils/Channel.hpp>

namespace rtt::net::utils {

enum ChannelType {
    UNDEFINED_CHANNEL,
    SETTINGS_CHANNEL,

    // AI to Robothub
    ROBOT_COMMANDS_BLUE_CHANNEL,
    ROBOT_COMMANDS_YELLOW_CHANNEL,
    SIMULATION_CONFIGURATION_CHANNEL,

    // Robothub to World
    ROBOT_FEEDBACK_CHANNEL,

    // World to AI
    WORLD_CHANNEL
};

extern const std::map<ChannelType, Channel> CHANNELS;

}  // namespace rtt::net::utils
