//
// Created by timovdk on 4/3/20.
//

#ifndef RTT_BALLMOVESSLOWGLOBALEVALUATION_H
#define RTT_BALLMOVESSLOWGLOBALEVALUATION_H

#include <NFParam/Param.h>

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates ball moves slow
 */
class BallMovesSlowGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Constructor for the BallMovesSlowGlobalEvaluation class
     */
    BallMovesSlowGlobalEvaluation() noexcept;

    /**
     * @brief Calculates the score for ball moves slow
     * @param world The current world
     * @param field The current field
     * @return The score of ball moves slow
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "BallMovesSlow"; }

   private:
    /**
     * @brief Calculates the actual metric value using the piecewise linear function member
     * @param x the x of the function
     * @return metric value between 0-255
     */
    [[nodiscard]] uint8_t calculateMetric(const double& x) const noexcept;

    std::unique_ptr<nativeformat::param::Param> piecewiseLinearFunction; /**< Unique pointer to the piecewise linear function that calculates the fuzzy value */
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLMOVESSLOWGLOBALEVALUATION_H
