//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_BASEEVALUATION_H
#define RTT_BASEEVALUATION_H

#include "world/World.hpp"

namespace rtt::ai::stp::evaluation {
/**
 * @brief Base class for evaluations
 */
class BaseEvaluation {
   public:
    /**
     * @brief Calculates the 'true-ness' of the invariant between 0 and 255. 0 == false, 255 == true
     * @param world the world
     * @param field the field
     * @return the 'true-ness' of this invariant during this tick
     */
    [[nodiscard]] virtual uint8_t metricCheck(const world::World *world, const Field *field) const noexcept = 0;

    /**
     * @brief Destructor of the BaseEvaluation class
     */
    virtual ~BaseEvaluation() = default;

    /**
     * @brief Retrieves the name of an evaluation
     * @return The name of an evaluation as string
     */
    virtual const char *getName() = 0;
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BASEEVALUATION_H
