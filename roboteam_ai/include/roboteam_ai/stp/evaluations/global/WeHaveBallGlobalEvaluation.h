//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_WEHAVEBALLGLOBALEVALUATION_H
#define RTT_WEHAVEBALLGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates we have ball
 */
class WeHaveBallGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for we have ball
     * @param world The current world
     * @param field The current field
     * @return The score of we have ball
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "WeHaveBall"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_WEHAVEBALLGLOBALEVALUATION_H
