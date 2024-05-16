#ifndef RTT_LINEOFSIGHTEVALUATION_H
#define RTT_LINEOFSIGHTEVALUATION_H

#include <cstdint>
#include <vector>

namespace rtt::ai::stp::evaluation {
/**
 * @brief Class that evaluates the Line of Sight score
 */
class LineOfSightEvaluation {
   public:
    /**
     * @brief Score line of sight to position based on enemy positions
     * @param receiverDistance Receiver distance from ball
     * @param enemyDistances Vector of all enemy distances from ball
     * @param enemyAngles Vector of all enemy angels from ball compared to Receiver angle (= 0 degrees)
     * @return uint8_t score
     */
    [[nodiscard]] static uint8_t metricCheck(double receiverDistance, std::vector<double>& enemyDistances, std::vector<double>& enemyAngles) noexcept;
};
}  // namespace rtt::ai::stp::evaluation
#endif  // RTT_LINEOFSIGHTEVALUATION_H
