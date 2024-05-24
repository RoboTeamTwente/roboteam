#include "stp/tactics/active/InstantKick.h"

#include "control/ControlUtils.h"
#include "stp/constants/ControlConstants.h"
#include "stp/skills/Kick.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

InstantKick::InstantKick() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Kick()}; }

std::optional<StpInfo> InstantKick::calculateInfoForSkill(const StpInfo &info) noexcept {
    if (!info.getPositionToShootAt() || !info.getRobot() || !info.getBall()) return std::nullopt;

    StpInfo skillStpInfo = info;
    double distanceBallToTarget = (info.getBall()->get()->position - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineKickForce(distanceBallToTarget, skillStpInfo.getShotType()));
    skillStpInfo.setDribblerOn(false);

    return skillStpInfo;
}

bool InstantKick::isEndTactic() noexcept { return false; }

bool InstantKick::isTacticFailing(const StpInfo &info) noexcept { return !info.getPositionToShootAt() || !info.getRobot()->get()->hasBall(); }

bool InstantKick::shouldTacticReset(const StpInfo &) noexcept { return false; }

const char *InstantKick::getName() { return "Instant Kick"; }

}  // namespace rtt::ai::stp::tactic