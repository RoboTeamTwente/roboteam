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


class InterfacePlay {
   public:
    std::atomic<bool> hasChanged;  // The idea is that the interface can read this without having to lock the mutex (In 99% of the cases, this will be false).
    InterfacePlay& operator= (const InterfacePlay&) = delete;

    inline void push(std::string playName) {
        std::lock_guard<std::mutex> guard(lock);
        name = std::move(playName);
        hasChanged = true;
    }

    inline std::string pop() {
        std::lock_guard<std::mutex> guard(lock);
        hasChanged = false;
        return std::move(name);
    }

   private:
    std::mutex lock;
    std::string name;
};

class RuntimeConfig {
   public:
    static inline InterfacePlay interfacePlay;
    static inline std::atomic<bool> useReferee;
    static inline std::atomic<bool> ignoreInvariants;
};

}  // namespace rtt::interface_new

#endif  // RTT_RUNTIMECONFIG_H
