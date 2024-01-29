//
// Created by jaro on 30-10-20.
//

#ifndef RTT_BALLONTHEIRSIDEGLOBALEVALUATION_H
#define RTT_BALLONTHEIRSIDEGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates ball on their side
 */
class BallOnTheirSideGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for ball on their side
     * @param world The current world
     * @return The score of ball on their side
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field*) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "BallOnTheirSideGlobalEvaluation"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLONTHEIRSIDEGLOBALEVALUATION_H
