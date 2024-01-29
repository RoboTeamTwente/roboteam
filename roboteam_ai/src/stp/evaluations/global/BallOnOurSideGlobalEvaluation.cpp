//
// Created by timovdk on 4/14/20.
/// T/F Invariant if the ball is on FRIENDLY side
//

#include "stp/evaluations/global/BallOnOurSideGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t BallOnOurSideGlobalEvaluation::metricCheck(const world::World* world, const Field*) const noexcept {
    return world->getWorld()->getBall().value()->position.x < 0 ? control_constants::FUZZY_TRUE : control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation