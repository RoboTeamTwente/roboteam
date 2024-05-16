#include "stp/evaluations/global/BallInOurDefenseAreaAndStillGlobalEvaluation.h"

#include "stp/constants/ControlConstants.h"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::evaluation {
uint8_t BallInOurDefenseAreaAndStillGlobalEvaluation::metricCheck(const world::World* world, const Field* field) const noexcept {
    return (field->leftDefenseArea.contains(world->getWorld()->getBall()->get()->position) &&
            field->leftDefenseArea.contains(world->getWorld()->getBall()->get()->expectedEndPosition))
               ? control_constants::FUZZY_TRUE
               : control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation