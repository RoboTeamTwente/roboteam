#pragma once
#include <map>
#include <string>
#include <utils/Channel.hpp>

namespace rtt::net::utils {

enum ChannelType {
    UNDEFINED_CHANNEL,
    GEOMETRY_CHANNEL,
    REFEREE_CHANNEL,
    WORLD_CHANNEL,
    ROBOT_COMMANDS_PRIMARY_CHANNEL,
    ROBOT_COMMANDS_SECONDARY_CHANNEL,
    ROBOT_FEEDBACK_PRIMARY_CHANNEL,
    ROBOT_FEEDBACK_SECONDARY_CHANNEL,
    SETTINGS_PRIMARY_CHANNEL,
    SETTINGS_SECONDARY_CHANNEL
};

extern const std::map<ChannelType, Channel> CHANNELS;

}  // namespace rtt::net::utils
