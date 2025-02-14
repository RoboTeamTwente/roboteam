#ifndef RTT_BallPlacementUsForceStart_H
#define RTT_BallPlacementUsForceStart_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

/**
 * @brief The ball placement us Force Start play is executed when the ball placement us game state is selected and the next ref command is not Free kick us, meaning it will be a
 * force start
 */
class BallPlacementUsForceStart : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    BallPlacementUsForceStart();

    /**
     * @brief Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::Field& field) noexcept override;

    /**
     * @brief Assigns robots to roles of this play
     * @return A map with assigned roles
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * @brief Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * @brief Retrieves the name of the play
     * @return The name of the play as a string
     */
    const char* getName() const override;

    void updateRoleConfiguration();

    static const int MANDATORY_ROLES = 2;

    mutable int numDefenders = 6;
    mutable int numWallers = 2;
    mutable int numAttackers = 1;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_BallPlacementUsForceStart_H