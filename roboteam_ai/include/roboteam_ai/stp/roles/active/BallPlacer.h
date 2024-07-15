#ifndef RTT_BALLPLACER_H
#define RTT_BALLPLACER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
/**
 * @brief Class that defines the ball placer role. The ball placer will try to place the ball at the ball placement position.
 */
class BallPlacer : public Role {
   public:
    /**
     * @brief Constructor that sets the name of the role and creates a state machine of tactics
     * @param name The name of the role
     */
    BallPlacer(std::string name);
    [[nodiscard]] Status update(StpInfo const& info) noexcept override;
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_BALLPLACER_H
