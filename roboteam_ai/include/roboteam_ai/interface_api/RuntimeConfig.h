//
// Created by Martin Miksik on 04/05/2023.
//

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

namespace rtt::ai::new_interface {

/**
 * The `InterfacePlay` class provides a thread-safe mechanism for setting play from interface
 */
class InterfacePlay {
   public:
    std::atomic<bool> hasChanged;  // The idea is that the interface can read this without having to lock the mutex (In 99% of the cases, this will be false).
    InterfacePlay& operator=(const InterfacePlay&) = delete;

    /**
     * @brief Pushes a new play name to the `InterfacePlay` object.
     *
     * @param playName The name of the play to be pushed.
     */
    inline void push(std::string playName) {
        std::lock_guard<std::mutex> guard(lock);
        name = std::move(playName);
        hasChanged = true;
    }

    /**
     * @brief Returns play name if it has changed since the last call to `pop()`.
     * Resets the `hasChanged` flag to false.
     *
     * @return The popped play name.
     */
    inline std::string pop() {
        std::lock_guard<std::mutex> guard(lock);
        hasChanged = false;
        return std::move(name);
    }

   private:
    std::mutex lock;
    std::string name;
};

/**
 * The `RuntimeConfig` class represents the runtime configuration settings. These can be updated only
 * from the interface as opposed to the `GameSettings`, which can be updated by the referee as well.
 * It provides static members to store and access various configuration options.
 */
class RuntimeConfig {
   public:
    static inline InterfacePlay interfacePlay;
    static inline std::atomic<bool> useReferee;
    static inline std::atomic<bool> ignoreInvariants;
};

}  // namespace rtt::ai::new_interface

#endif  // RTT_RUNTIMECONFIG_H
