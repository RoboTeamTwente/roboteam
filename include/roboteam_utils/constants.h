#pragma once

#include <array>
#include <string>
#include <roboteam_proto/Channel.h>

namespace roboteam_utils {

extern const roboteam_proto::Channel GEOMETRY_CHANNEL;
extern const roboteam_proto::Channel REFEREE_CHANNEL;
extern const roboteam_proto::Channel WORLD_CHANNEL;
extern const roboteam_proto::Channel ROBOT_COMMANDS_PRIMARY_CHANNEL;
extern const roboteam_proto::Channel ROBOT_COMMANDS_SECONDARY_CHANNEL;
extern const roboteam_proto::Channel FEEDBACK_PRIMARY_CHANNEL;
extern const roboteam_proto::Channel FEEDBACK_SECONDARY_CHANNEL;
extern const roboteam_proto::Channel SETTINGS_PRIMARY_CHANNEL;
extern const roboteam_proto::Channel SETTINGS_SECONDARY_CHANNEL;

} // namespace rtt