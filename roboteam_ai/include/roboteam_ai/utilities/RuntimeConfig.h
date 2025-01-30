#ifndef RTT_RUNTIMECONFIG_H
#define RTT_RUNTIMECONFIG_H

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "roboteam_utils/Print.h"
#include "utilities/GameState.h"

namespace rtt::ai {
/**
 * The `RuntimeConfig` class represents the runtime configuration settings. These can be updated only
 * from the interface as opposed to the `GameSettings`, which can be updated by the referee as well.
 * It provides static members to store and access various configuration options.
 */
class RuntimeConfig {
   public:
    static inline std::atomic<bool> useReferee = true;
    static inline std::atomic<bool> ignoreInvariants;
    static inline std::atomic<bool> isPaused;
};

}  // namespace rtt::ai

#endif  // RTT_RUNTIMECONFIG_H
