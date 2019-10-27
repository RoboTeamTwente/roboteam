#include "constants.h"

namespace roboteam_utils {

template<typename T, long unsigned int N>
static bool has(std::array<T, N> arr, T val) {
  for (unsigned int i = 0; i < N; i++) {
    if (arr[i] == val) {
      return true;
    }
  }
  return false;
}

const std::map<ChannelType, roboteam_proto::Channel> CHANNELS = {
    {GEOMETRY_CHANNEL, {"geometry", "tcp://127.0.0.1:5556"}},
    {REFEREE_CHANNEL, {"referee", "tcp://127.0.0.1:5557"}},
    {WORLD_CHANNEL, {"world", "tcp://127.0.0.1:5558"}},
    {ROBOT_COMMANDS_PRIMARY_CHANNEL, {"commands_primary", "tcp://127.0.0.1:5559"}},
    {ROBOT_COMMANDS_SECONDARY_CHANNEL, {"commands_secondary", "tcp://127.0.0.1:5560"}},
    {FEEDBACK_PRIMARY_CHANNEL, {"feedback_primary", "tcp://127.0.0.1:5561"}},
    {FEEDBACK_SECONDARY_CHANNEL, {"feedback_yellow", "tcp://127.0.0.1:5562"}},
    {SETTINGS_PRIMARY_CHANNEL, {"settings_primary", "tcp://127.0.0.1:5563"}},
    {SETTINGS_SECONDARY_CHANNEL, {"settings_secondary", "tcp://127.0.0.1:5564"}}
};

} // namespace rtt
