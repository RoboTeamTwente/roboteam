#ifndef RTT_CHANNELS_H
#define RTT_CHANNELS_H

#include <map>
#include <string>

#include "Channel.h"

namespace proto {

enum ChannelType {
    UNDEFINED_CHANNEL,
    GEOMETRY_CHANNEL,
    REFEREE_CHANNEL,
    WORLD_CHANNEL,
    ROBOT_COMMANDS_PRIMARY_CHANNEL,
    ROBOT_COMMANDS_SECONDARY_CHANNEL,
    FEEDBACK_PRIMARY_CHANNEL,
    FEEDBACK_SECONDARY_CHANNEL,
    SETTINGS_PRIMARY_CHANNEL,
    SETTINGS_SECONDARY_CHANNEL
};

extern const std::map<ChannelType, Channel> CHANNELS;
}  // namespace proto
#endif  // RTT_CHANNELS_H
