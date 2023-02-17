//
// Created by roboteam on 23/6/20.
//

#ifndef RTT_BALLCLOSESTTOUSGLOBALEVALUATION_H
#define RTT_BALLCLOSESTTOUSGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates ball closest to us
 */
class BallClosestToUsGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for ball closest to us
     * @param world The current world
     * @param field The current field
     * @return The score of ball closest to us
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "BallClosestToUs"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLCLOSESTTOUSGLOBALEVALUATION_H
