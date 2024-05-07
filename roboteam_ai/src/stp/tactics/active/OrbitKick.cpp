#include "stp/tactics/active/OrbitKick.h"

#include "control/ControlUtils.h"
#include "stp/skills/Kick.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

OrbitKick::OrbitKick() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Rotate(), skill::Kick()}; }

std::optional<StpInfo> OrbitKick::calculateInfoForSkill(const StpInfo &info) noexcept {
    if (!info.getPositionToShootAt() || !info.getRobot() || !info.getBall()) return std::nullopt;

    StpInfo skillStpInfo = info;
    double angleToTarget = (info.getPositionToShootAt().value() - info.getRobot().value()->getPos()).angle();
    skillStpInfo.setAngle(angleToTarget);

    double distanceBallToTarget = (info.getBall()->get()->position - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineKickForce(distanceBallToTarget, skillStpInfo.getShotType()));

    skillStpInfo.setDribblerSpeed(100);

    return skillStpInfo;
}

bool OrbitKick::isEndTactic() noexcept { return false; }

bool OrbitKick::isTacticFailing(const StpInfo &info) noexcept { return !info.getPositionToShootAt() || !info.getRobot()->get()->hasBall(); }

bool OrbitKick::shouldTacticReset(const StpInfo &) noexcept { return false; }

const char *OrbitKick::getName() { return "Orbit Kick"; }

}  // namespace rtt::ai::stp::tactic