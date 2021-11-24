#pragma once
#include <map>
#include <string>
#include <utils/Channel.hpp>

namespace rtt::net::utils {

enum ChannelType {
    UNDEFINED_CHANNEL,
    WORLD_CHANNEL,
    ROBOT_COMMANDS_CHANNEL,
    ROBOT_FEEDBACK_CHANNEL,
    SETTINGS_CHANNEL
};

extern const std::map<ChannelType, Channel> CHANNELS;

}  // namespace rtt::net::utils
