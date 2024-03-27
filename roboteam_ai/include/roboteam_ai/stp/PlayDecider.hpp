#ifndef RTT_PLAYDECIDER_HPP
#define RTT_PLAYDECIDER_HPP

#include <atomic>
#include <mutex>

#include "Play.hpp"

namespace rtt::ai::stp {

struct PlayLock {
    std::atomic<bool> didChange; /**< Bool indicating if the current play was changed using lockPlay method */
    std::atomic<bool> isSet;     /**< Bool indicating if the current play was set using lockPlay method */

    std::optional<std::string> playName; /**< The play name that is set in the interface */
    std::mutex lock;                     /**< Mutex to lock the interfacePlayStr */
};

/**
 * @brief Class that defines the play decider. It decides the best play from a vector of plays.
 * If there is a play set in the interface, this play will be picked.
 */
class PlayDecider {
    static inline PlayLock playLock; /**< The play that is locked in the interface */

   public:
    /**
     * @brief Returns if the play was changed using the lockPlay method. Also resets the didChange bool to false
     * @return true if the play was changed using the lockPlay, false otherwise
     */
    static bool didLockPlay() noexcept;

    /**
     * @brief Sets play name to lock
     * @param playName playName to lock to
     */
    static void lockPlay(const std::optional<std::string> playName);

    /**
     * @brief Unlocks the play
     */
    static void unlockPlay();

    /**
     * @brief This function checks if there is a locked play. If there is, pick that play.
     * If there isn't, pick the play with the highest score (either a locked play through the interface or just the highest scored play)
     * @param World The world pointer
     * @param plays the vector of plays
     * @return The best play for the current tick
     */
    static Play* decideBestPlay(const rtt::world::World* world, const std::vector<std::unique_ptr<ai::stp::Play>>& plays) noexcept;

    /**
     * @brief Returns (a pointer to) a play with a given name
     * @param name name of the play. Should exactly match the name returned by the getName() method in the play
     * @param plays all plays to check the name for
     * @return the play with the given name, or a nullptr if no such play is found
     */
    static Play* getPlayForName(std::string name, const std::vector<std::unique_ptr<ai::stp::Play>>& plays);
};
}  // namespace rtt::ai::stp

#endif  // RTT_PLAYDECIDER_HPP
