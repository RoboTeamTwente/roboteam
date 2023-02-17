//
// Created by RobotJesse on 08-04-20.
//

#ifndef RTT_AVOIDBALL_H
#define RTT_AVOIDBALL_H

#include "roboteam_utils/Shape.h"
#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
/**
 * Class that defines the avoid ball tactic. This tactic is used when the robot is not allowed to be near the ball
 */
class AvoidBall : public Tactic {
   public:
    /**
     * @brief Constructor for the tactic, it constructs the state machine of skills
     */
    AvoidBall();

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
     * @brief Checks whether this is a passive tactic (formerly called endTactic)
     * @return This will always return true, since it is an endTactic
     */
    bool isEndTactic() noexcept override;

    /**
     * @brief Computes a new location to go to that is outside of the avoidShape and a valid position
     * or the original targetPos is no such position is found (although this is very unlikely).
     * @param targetPos the position the robot would like to go to
     * @param field the current field
     * @param avoidShape the area in the field that needs to be avoided
     * @return new position to go to which is valid and outside the avoidShape,
     */
    Vector2 calculateNewPosition(Vector2 targetPos, const rtt::Field &field, const std::unique_ptr<Shape> &avoidShape);

    /**
     * @brief Gets the tactic name
     * @return The name of this tactic
     */
    const char *getName() override;
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_AVOIDBALL_H
