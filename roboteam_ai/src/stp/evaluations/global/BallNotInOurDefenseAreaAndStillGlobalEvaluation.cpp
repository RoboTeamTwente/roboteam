//
// Created by Alexander on 29-01-2022
/// T/F Invariant if the ball is in our defense area and not moving
//

#include "stp/evaluations/global/BallNotInOurDefenseAreaAndStillGlobalEvaluation.h"

#include "stp/constants/ControlConstants.h"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::evaluation {
uint8_t BallNotInOurDefenseAreaAndStillGlobalEvaluation::metricCheck(const world::World* world, const Field* field) const noexcept {
    return (field->leftDefenseArea.contains(world->getWorld()->getBall()->get()->position) &&
            field->leftDefenseArea.contains(world->getWorld()->getBall()->get()->expectedEndPosition))
               ? control_constants::FUZZY_FALSE
               : control_constants::FUZZY_TRUE;
}
}  // namespace rtt::ai::stp::evaluation