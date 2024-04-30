#include "stp/tactics/active/InstantKick.h"

#include "control/ControlUtils.h"
#include "stp/constants/ControlConstants.h"
#include "stp/skills/Kick.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

InstantKick::InstantKick() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Kick()};
}

std::optional<StpInfo> InstantKick::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getPositionToShootAt() || !skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    // Calculate the distance and the kick force
    double distanceBallToTarget = (info.getBall()->get()->position - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineKickForce(distanceBallToTarget, skillStpInfo.getShotType()));

    skillStpInfo.setDribblerSpeed(0);

    return skillStpInfo;
}

bool InstantKick::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool InstantKick::isTacticFailing(const StpInfo &info) noexcept {
    // Fail tactic if there is no shootTarget or we don't have the ball
    if (!info.getPositionToShootAt() || !info.getRobot()->get()->hasBall()) return true;
    return false;
}

bool InstantKick::shouldTacticReset(const StpInfo &) noexcept { return false; }

const char *InstantKick::getName() { return "Instant Kick"; }

}  // namespace rtt::ai::stp::tactic
