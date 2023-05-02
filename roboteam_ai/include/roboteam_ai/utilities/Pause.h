//
// Created by baris on 15-2-19.
//

#ifndef ROBOTEAM_AI_PAUSE_H
#define ROBOTEAM_AI_PAUSE_H

#include <mutex>

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
    static bool pause; /**< Indicates whether the robots are paused */
    static std::mutex pauseLock; /**< Synchronizer for the pause button */

    /**
     * @brief Constructor for the Pause class. No instance should ever be needed
     */
    Pause();

   public:
    /**
     * @brief Getter for the pause boolean
     * @return Whether the robots are paused
     */
    static bool getPause();

    /**
     * @brief Halts the robots
     * @param data The current world
     */
    static void haltRobots(rtt::world::World const* data);

    /**
     * @brief Sets the pause boolean
     * @param set The value to which the pause boolean should be set
     */
    static void setPause(bool set);
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_PAUSE_H
