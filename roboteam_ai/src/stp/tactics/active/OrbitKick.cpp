#include "stp/tactics/active/OrbitKick.h"

#include "control/ControlUtils.h"
#include "stp/skills/Kick.h"
#include "stp/skills/OrbitAngular.h"

namespace rtt::ai::stp::tactic {

OrbitKick::OrbitKick() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{
    skill::OrbitAngular(), 
    skill::Kick()}; }

std::optional<StpInfo> OrbitKick::calculateInfoForSkill(const StpInfo &info) noexcept {
    if (!info.getPositionToShootAt() || !info.getRobot() || !info.getBall()) return std::nullopt;

    StpInfo skillStpInfo = info;
    double angleToTarget = (info.getPositionToShootAt().value() - info.getRobot().value()->getPos()).angle();
    skillStpInfo.setYaw(angleToTarget);

    double distanceBallToTarget = (info.getBall()->get()->position - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineKickForce(distanceBallToTarget, skillStpInfo.getShotPower()));

    skillStpInfo.setDribblerOn(true);

    return skillStpInfo;
}

bool OrbitKick::isEndTactic() noexcept { return false; }

bool OrbitKick::isTacticFailing(const StpInfo &info) noexcept { return !info.getPositionToShootAt() || !info.getRobot()->get()->hasBall(); }

bool OrbitKick::shouldTacticReset(const StpInfo &info) noexcept {
    const auto errorMargin = constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    return info.getRobot().value()->getYaw().shortestAngleDiff(info.getYaw()) > errorMargin;
}

const char *OrbitKick::getName() { return "Orbit Kick"; }

}  // namespace rtt::ai::stp::tactic