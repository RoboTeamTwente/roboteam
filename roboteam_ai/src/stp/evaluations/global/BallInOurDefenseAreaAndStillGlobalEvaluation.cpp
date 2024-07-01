#include "stp/evaluations/global/BallInOurDefenseAreaAndStillGlobalEvaluation.h"

#include "utilities/Constants.h"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::evaluation {
uint8_t BallInOurDefenseAreaAndStillGlobalEvaluation::metricCheck(const world::World* world, const Field* field) const noexcept {
    double ballRadius = constants::BALL_RADIUS / 2;
    auto ourDefenseArea = FieldComputations::getDefenseArea(*field, true, ballRadius, 0);
    return (ourDefenseArea.contains(world->getWorld()->getBall()->get()->position) && ourDefenseArea.contains(world->getWorld()->getBall()->get()->expectedEndPosition))
               ? constants::FUZZY_TRUE
               : constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation