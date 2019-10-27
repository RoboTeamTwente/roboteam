#pragma once

#include <array>
#include <string>
#include <map>

#include <roboteam_proto/Channel.h>

namespace roboteam_utils {

enum ChannelType {
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

extern const std::map<ChannelType, roboteam_proto::Channel> CHANNELS;


} // namespace rtt