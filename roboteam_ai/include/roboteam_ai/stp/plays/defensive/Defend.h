#ifndef RTT_DEFEND_H
#define RTT_DEFEND_H

#include <stp/computations/InterceptionComputations.h>

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

/**
 * @brief Defend Play is executed when the opponent has or is close to the ball.
 * In this case they most likely will try to score. Some defenders defend the goal by blocking the path between enemy
 * robots and the goal. Other defenders block other enemy robots to avoid passes to them.
 */
class Defend : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    Defend();

    /**
     * @brief Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::Field& field) noexcept override;

    /**
     * @brief Assigns robots to roles of this play
     * @return Map with assigned roles
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * @brief Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * @brief Gets the play name
     */
    const char* getName() const override;

    InterceptionInfo harasserInfo;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_DEFEND_H
