//
// Created by jibbe on 5-6-23.
//

#ifndef RTT_ONETWOATTACK_HPP
#define RTT_ONETWOATTACK_HPP

#include "stp/Play.hpp"
#include "stp/computations/PassComputations.h"

namespace rtt::ai::stp::play {

class OneTwoAttack : public Play {
   public:
    PassInfo firstPassInfo; /**< Struct containing info about the first pass. Pass to a location in one of the two offensive corner grids */
    PassInfo secondPassInfo; /**< Struct containing info about the first pass. Pass to a location from where the striker can shoot */

    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    OneTwoAttack();

    /**
     * @brief Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::Field &field) noexcept override;

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
     * @brief Retrieves the name of the play
     * @return The name of the play as string
     */
    const char *getName() override;

    /**
     * @brief Check if play should end. True when attacker role is finished.
     */
    bool shouldEndPlay() noexcept override;

    /**
     * @brief calculates info for the keeper
     */
    void calculateInfoForKeeper() noexcept;

    /**
     * @brief calculates info for the striker
     */
    void calculateInfoForStriker() noexcept;

    /**
     * @brief calculates info for the assistant
     */
    void calculateInfoForAssistant() noexcept;

    /**
     * @brief calculates info for the wallers
     */
    void calculateInfoForWallers() noexcept;

    /**
     * @brief calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * @brief determines whether the ball has been kicked or not
     * @return boolean that tells whether the ball has been kicked
     */
    bool ballKicked();

    void reset() override;
};
}  // namespace rtt::ai::stp::play
#endif  // RTT_ONETWOATTACK_HPP
