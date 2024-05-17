#ifndef RTT_BLOCKBALL_H
#define RTT_BLOCKBALL_H

#include "stp/Tactic.h"
#include "utilities/StpInfoEnums.h"

namespace rtt::ai::stp::tactic {
/**
 * @brief Class that defines the block ball tactic. This tactic is used for blocking the ball from going to a position
 */
class BlockBall : public Tactic {
   public:
    /**
     * @brief Constructor for the tactic, it constructs the state machine of skills
     */
    BlockBall();

    /**
     * @brief Calculates the position for the blocker
     * @param ball Ball
     * @param defendPos Position to defend
     * @return Target position for the blocker
     */
    static Vector2 calculateTargetPosition(const world::view::BallView &ball, Vector2 defendPos) noexcept;

   private:
    /**
     * @brief Calculate the info for skills from the StpInfo struct parameter
     * @param info info is the StpInfo passed by the role
     * @return std::optional<SkillInfo> based on the StpInfo parameter
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * @brief Is this tactic failing during execution (go back to the previous tactic)
     * This tactic can never fail, so always returns false
     * @param info StpInfo can be used to check some data
     * @return true, tactic will fail (go back to prev tactic), false execution will continue as usual
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * @brief Should this tactic be reset (go back to the first skill of this tactic)
     * Returns true when the robot is further away from the target position than some error margin
     * @param info StpInfo can be used to check some data
     * @return true if tactic  should reset, false if execution should continue
     */
    bool shouldTacticReset(const StpInfo &info) noexcept override;

    /**
     * @brief Checks whether this is an end tactic, meaning it should keep looping this tactic if all skills are finished
     * @return This will always return true, since it is an endTactic
     */
    bool isEndTactic() noexcept override;

    /**
     * @brief Gets the tactic name
     * @return The name of this tactic
     */
    const char *getName() override;
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_BLOCKBALL_H
