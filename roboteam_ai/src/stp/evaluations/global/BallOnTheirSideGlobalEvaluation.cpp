#include "stp/evaluations/global/BallOnTheirSideGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t BallOnTheirSideGlobalEvaluation::metricCheck(const world::World* world, const Field*) const noexcept {
    return world->getWorld()->getBall().value()->position.x >= 0 ? control_constants::FUZZY_TRUE : control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation