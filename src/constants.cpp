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

const roboteam_proto::Channel GEOMETRY_CHANNEL =                  {"geometry", "tcp://127.0.0.1:5556"};
const roboteam_proto::Channel REFEREE_CHANNEL =                   {"referee", "tcp://127.0.0.1:5557"};
const roboteam_proto::Channel WORLD_CHANNEL =                     {"world", "tcp://127.0.0.1:5558"};
const roboteam_proto::Channel ROBOT_COMMANDS_PRIMARY_CHANNEL =    {"commands_primary", "tcp://127.0.0.1:5559"};
const roboteam_proto::Channel ROBOT_COMMANDS_SECONDARY_CHANNEL =  {"commands_secondary", "tcp://127.0.0.1:5560"};
const roboteam_proto::Channel FEEDBACK_PRIMARY_CHANNEL =          {"feedback_primary", "tcp://127.0.0.1:5561"};
const roboteam_proto::Channel FEEDBACK_SECONDARY_CHANNEL =        {"feedback_yellow", "tcp://127.0.0.1:5562"};
const roboteam_proto::Channel SETTINGS_PRIMARY_CHANNEL=           {"settings_primary", "tcp://127.0.0.1:5563"};
const roboteam_proto::Channel SETTINGS_SECONDARY_CHANNEL=         {"settings_secondary", "tcp://127.0.0.1:5564"};

} // namespace rtt
