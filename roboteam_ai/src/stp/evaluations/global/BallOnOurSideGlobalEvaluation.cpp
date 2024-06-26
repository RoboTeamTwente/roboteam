#include "stp/evaluations/global/BallOnOurSideGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t BallOnOurSideGlobalEvaluation::metricCheck(const world::World* world, const Field*) const noexcept {
    return world->getWorld()->getBall().value()->position.x < 0 ? constants::FUZZY_TRUE : constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation