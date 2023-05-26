//
// Created by jordi on 24-03-20.
//

#ifndef RTT_ATTACK_H
#define RTT_ATTACK_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

/**
 * @brief The attack play is executed when there is a chance to shoot at the enemey goal
 */
class Attack : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    Attack();

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
     * @brief Calculate info for the roles that need to be calculated for scoring
     */
    void calculateInfoForScoredRoles(world::World*) noexcept override{};

    /**
     * @brief Check if play should end. True when attacker role is finished.
     */
    bool shouldEndPlay() noexcept override;

    /**
     * @brief Retrieves the name of the play
     * @return The name of the play as string
     */
    const char* getName() const override;

    /**
     * @brief calculates info for the blocker roll
     */
    void calculateInfoForBlocker() noexcept;
};

}  // namespace rtt::ai::stp::play

#endif  // RTT_ATTACK_H
