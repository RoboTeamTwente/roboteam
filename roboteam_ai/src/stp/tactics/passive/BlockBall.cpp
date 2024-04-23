#include "stp/tactics/passive/BlockBall.h"

#include <roboteam_utils/Circle.h>

#include "stp/computations/PositionComputations.h"
#include "stp/skills/GoToPos.h"

namespace rtt::ai::stp::tactic {

BlockBall::BlockBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> BlockBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getField() || !skillStpInfo.getBall() || !skillStpInfo.getRobot() || !(skillStpInfo.getPositionToDefend() || skillStpInfo.getPositionToMoveTo()))
        return std::nullopt;

    if (!skillStpInfo.getPositionToMoveTo()) {
        auto defendPos = info.getPositionToDefend().value();
        auto targetPosition = calculateTargetPosition(info.getBall().value(), defendPos);

        // Make sure this position is valid
        targetPosition = FieldComputations::projectPointToValidPositionOnLine(info.getField().value(), targetPosition, defendPos, info.getBall()->get()->position);
        // targetPosition.x = std::min(0.0, targetPosition.x);

        skillStpInfo.setPositionToMoveTo(targetPosition);

        auto targetAngle = (info.getBall()->get()->position - info.getRobot()->get()->getPos()).angle();
        skillStpInfo.setAngle(targetAngle);
    } else {
        skillStpInfo.setDribblerSpeed(0);
    }

    return skillStpInfo;
}

bool BlockBall::isEndTactic() noexcept { return true; }

bool BlockBall::isTacticFailing(const StpInfo &) noexcept { return false; }

bool BlockBall::shouldTacticReset(const StpInfo &) noexcept { return false; }

const char *BlockBall::getName() { return "Block Ball"; }

Vector2 BlockBall::calculateTargetPosition(const world::view::BallView &ball, Vector2 defendPos) noexcept {
    return defendPos + (ball->position - defendPos).stretchToLength(4 * control_constants::ROBOT_RADIUS);
}

}  // namespace rtt::ai::stp::tactic