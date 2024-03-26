#ifndef RTT_WeWILLHAVEBALLGLOBALEVALUATION_H
#define RTT_WEWILLHAVEBALLGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates whether we will have the ball
 */
class WeWillHaveBallGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for we will have ball
     * @param world The current world
     * @return The score of we will have ball
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field*) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "WeWillHaveBall"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_WEWILLHAVEBALLGLOBALEVALUATION_H
