//
// Created by timovdk on 3/12/20.
/// Rotates (with dribbler) the robot if needed to target angle then calculates the power to KICK the BALL with.
// TODO-Max compare to ChipAtPos

/// ACTIVE
//

#include "stp/tactics/active/KickAtPos.h"

#include "control/ControlUtils.h"
#include "stp/constants/ControlConstants.h"
#include "stp/skills/Kick.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

KickAtPos::KickAtPos() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Rotate(), skill::Kick()};
}

std::optional<StpInfo> KickAtPos::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getPositionToShootAt() || !skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    // Calculate the angle the robot needs to aim
    double angleToTarget = (info.getPositionToShootAt().value() - info.getRobot().value()->getPos()).angle();
    skillStpInfo.setAngle(angleToTarget);

    // Calculate the distance and the kick force
    double distanceBallToTarget = (info.getBall()->get()->position - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineKickForce(distanceBallToTarget, skillStpInfo.getShotType()));

    skillStpInfo.setDribblerSpeed(100);

    return skillStpInfo;
}

bool KickAtPos::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool KickAtPos::isTacticFailing(const StpInfo &info) noexcept {
    // Fail tactic if there is no shootTarget or we don't have the ball
    if (!info.getPositionToShootAt() || !info.getRobot()->get()->hasBall()) return true;
    return false;
}

bool KickAtPos::shouldTacticReset(const StpInfo &info) noexcept { return false; }

const char *KickAtPos::getName() { return "Kick At Pos"; }

}  // namespace rtt::ai::stp::tactic
