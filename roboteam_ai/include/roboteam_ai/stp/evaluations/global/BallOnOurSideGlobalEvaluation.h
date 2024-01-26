//
// Created by timovdk on 4/14/20.
//

#ifndef RTT_BALLONOURSIDEGLOBALEVALUATION_H
#define RTT_BALLONOURSIDEGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates ball on our side
 */
class BallOnOurSideGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for ball on our side
     * @param world The current world
     * @return The score of ball on our side
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field*) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "BallOnOurSideGlobalEvaluation"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLONOURSIDEGLOBALEVALUATION_H
