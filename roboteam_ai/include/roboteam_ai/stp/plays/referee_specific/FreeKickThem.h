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
     * @brief Retrieves the name of the play
     * @return The name of the play
     */
    const char *getName() const override;

   protected:
    /**
     * @brief Calculates info for the harasser
     */
    void calculateInfoForHarasser() noexcept;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_FREEKICKTHEM_H
