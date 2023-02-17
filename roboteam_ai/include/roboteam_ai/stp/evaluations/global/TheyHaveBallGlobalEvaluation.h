//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_THEYHAVEBALLGLOBALEVALUATION_H
#define RTT_THEYHAVEBALLGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates they have ball
 */
class TheyHaveBallGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for they have ball
     * @param world The current world
     * @param field The current field
     * @return The score of they have ball
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "TheyHaveBall"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_THEYHAVEBALLGLOBALEVALUATION_H
