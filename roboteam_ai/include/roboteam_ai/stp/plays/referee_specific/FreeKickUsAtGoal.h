//
// Created by jordi on 24-03-20.
//

#ifndef RTT_FREEKICKUSATGOAL_H
#define RTT_FREEKICKUSATGOAL_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {
/**
 * @brief The free kick us at goal play is executed when we can shoot at the goal and the free kick us game state is selected
 */
class FreeKickUsAtGoal : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    FreeKickUsAtGoal();

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
     * @brief Calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * @brief Calculates info for the midfielders
     */
    void calculateInfoForMidfielders() noexcept;

    /**
     * @brief Calculates info for the attackers
     */
    void calculateInfoForAttackers() noexcept;

    /**
     * @brief Check if play should end. True when the free kick taker has kicked the ball
     */
    bool shouldEndPlay() noexcept override;

    /**
     * @brief Retrieves the name of the play
     * @return The name of the play as a string
     */
    const char* getName() override;
};

}  // namespace rtt::ai::stp::play

#endif  // RTT_FREEKICKUSATGOAL_H
