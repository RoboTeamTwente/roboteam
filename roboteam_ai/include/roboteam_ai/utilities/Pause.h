//
// Created by baris on 15-2-19.
//

#ifndef ROBOTEAM_AI_PAUSE_H
#define ROBOTEAM_AI_PAUSE_H

#include <mutex>
#include <optional>
#include "world/views/WorldDataView.hpp"

namespace rtt::world {
class World;
}

namespace rtt::ai {

namespace io {
class IOManager;
}

/**
 * @brief Class that allows us to pause the robots from the interface.
 */
class Pause {
   private:
    static std::atomic<bool> state;

   public:
    Pause() = delete;
    static bool isPaused();
    static void pause(std::optional<rtt::world::view::WorldDataView> data);
    static void resume();
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_PAUSE_H
