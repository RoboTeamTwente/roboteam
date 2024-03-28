#ifndef RTT_BALLINOURDEFENSEAREAANDSTILL_H
#define RTT_BALLINOURDEFENSEAREAANDSTILL_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates ball in our defense area and still
 */
class BallInOurDefenseAreaAndStillGlobalEvaluation : public BaseEvaluation {
   public:
    /**
     * @brief Calculates the score for ball in our defense area and still
     * @param world The current world
     * @param field The current field
     * @return The score of ball in our defense area and still
     */
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const Field* field) const noexcept override;

    /**
     * @brief Retrieves the name of the evaluation
     * @return A string containing the name of the evaluation
     */
    const char* getName() override { return "BallInOurDefenseAreaAndStillGlobalEvaluation"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLINOURDEFENSEAREAANDSTILL_H
