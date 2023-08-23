#ifndef RTT_BALLPLACEMENTTHEM_H
#define RTT_BALLPLACEMENTTHEM_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

/**
 * @brief The ball placement them play is executed when the ball placement them game state is selected
 */
class BallPlacementThem : public Play {
   public:
    /**
     * @brief Constructor that initializes roles with roles that are necessary for this play
     */
    BallPlacementThem();

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

    void calculateInfoForKeeper() noexcept;

    /**
     * @brief Calculates info for the harasser roles
     */
    void calculateInfoForHarasser() noexcept;

    /**
     * @brief Calculates info for the waller roles
     */
    void calculateInfoForWallers() noexcept;

    /**
     * @brief Retrieves the name of the play
     * @return The name of the play as string
     */
    const char* getName() const override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_BALLPLACEMENTTHEM_H