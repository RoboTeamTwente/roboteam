//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_WEDONOTHAVEBALLGLOBALEVALUATION_H
#define RTT_WEDONOTHAVEBALLGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates we don't have ball
 */
class WeDoNotHaveBallGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for we don't have ball
     * @param world The current world
     * @return The score of we don't have ball
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field*) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "WeDoNotHaveBall"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_WEDONOTHAVEBALLGLOBALEVALUATION_H
