//
// Created by Alexander on 29-01-2022
//

#ifndef RTT_BALLNOTINOURDEFENSEAREAANDSTILL_H
#define RTT_BALLNOTINOURDEFENSEAREAANDSTILL_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates ball not in our defense area and still
 */
class BallNotInOurDefenseAreaAndStillGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for ball not in our defense area and still
     * @param world The current world
     * @param field The current field
     * @return The score of ball not in our defense area and still
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "BallNotInOurDefenseAreaAndStillGlobalEvaluation"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLNOTINOURDEFENSEAREAANDSTILL_H
