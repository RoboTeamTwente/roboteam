//
// Created by Alexander on 27-03-20.
//

#ifndef RTT_THEYDONOTHAVEBALLGLOBALEVALUATION_H
#define RTT_THEYDONOTHAVEBALLGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates they do not have ball
 */
class TheyDoNotHaveBallGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for they do not have ball
     * @param world The current world
     * @return The score of they do not have ball
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field*) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "TheyDoNotHaveBall"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_THEYDONOTHAVEBALLGLOBALEVALUATION_H
