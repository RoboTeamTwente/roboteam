//
// Created by jordi on 28-04-20.
//

#ifndef RTT_BALLISFREEGLOBALEVALUATION_H
#define RTT_BALLISFREEGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates ball is free
 */
class BallIsFreeGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for ball is free
     * @param world The current world
     * @param field The current field
     * @return The score of ball is free
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "BallIsFree"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLISFREEGLOBALEVALUATION_H
