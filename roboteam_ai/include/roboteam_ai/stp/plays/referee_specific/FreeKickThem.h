//
// Created by jordi on 07-05-20.
//

#ifndef RTT_FREEKICKTHEM_H
#define RTT_FREEKICKTHEM_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {
/**
 * @brief The free kick them play is executed when the free kick them game state is selected
 */
class FreeKickThem : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    FreeKickThem();

    /**
     * @brief Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::Field &field) noexcept override;

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
     * @brief Calculate info for the roles that need to be calculated for scoring
     */
    void calculateInfoForScoredRoles(world::World *) noexcept override{};

    /**
     * @brief Retrieves the name of the play
     * @return The name of the play
     */
    const char *getName() const override;

   protected:
    /**
     * @brief Calculates info for the wallers
     */
    void calculateInfoForWallers() noexcept;

    /**
     * @brief Calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * @brief Calculates info for the ballBlocker
     */
    void calculateInfoForBlocker() noexcept;

    /**
     * @brief Calculates info for the harasser
     */
    void calculateInfoForHarasser() noexcept;

    /**
     * @brief Calculates info for the keeper
     */
    void calculateInfoForKeeper() noexcept;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_FREEKICKTHEM_H
