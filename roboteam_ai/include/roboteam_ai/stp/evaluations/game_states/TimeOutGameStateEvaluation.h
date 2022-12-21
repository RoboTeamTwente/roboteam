//
// Created by jordi on 28-04-20.
//

#ifndef RTT_TIMEOUTGAMESTATEEVALUATION_H
#define RTT_TIMEOUTGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the time out game state
 */
class TimeOutGameStateEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for the time out game state
     * @param world The current world
     * @param field The current field
     * @return The score of the time out game state
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "gs::TimeOut"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_TIMEOUTGAMESTATEEVALUATION_H
