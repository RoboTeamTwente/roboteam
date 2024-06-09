#ifndef RTT_GOALCOMPUTATIONS_H
#define RTT_GOALCOMPUTATIONS_H

#include <roboteam_utils/LineSegment.h>

#include <roboteam_utils/Field.hpp>
#include <world/World.hpp>

namespace rtt::ai::stp::computations {

/**
 * @brief Class with computations about shooting at the goal
 */
class GoalComputations {
   public:
    /**
     * @brief Calculate point in goal to aim for
     * @return Target point
     */
    static Vector2 calculateGoalTarget(rtt::world::World *world, const rtt::Field &field);

    /**
     * @brief Calculate points we want to aim for
     * @param field Field
     * @param fromPoint Position to shoot from
     * @return Line between the two aim points
     */
    static rtt::LineSegment getAimPoints(const rtt::Field &field, const rtt::Vector2 &sourcePoint);

    /**
     * @brief Returns the longest line from openSegments
     * @param openSegments Vector of lines
     * @return Longest line from openSegments
     */
    static const LineSegment getLongestSegment(const std::vector<LineSegment> &openSegments);
};
}  // namespace rtt::ai::stp::computations
#endif  // RTT_GOALCOMPUTATIONS_H
