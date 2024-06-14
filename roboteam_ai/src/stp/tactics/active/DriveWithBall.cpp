#include "stp/tactics/active/DriveWithBall.h"

#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

DriveWithBall::DriveWithBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Rotate(), skill::GoToPos()}; }

std::optional<StpInfo> DriveWithBall::calculateInfoForSkill(const StpInfo &info) noexcept {
    if (!info.getPositionToMoveTo() || !info.getBall()) return std::nullopt;

    StpInfo skillStpInfo = info;
    auto angleToTarget = (info.getPositionToMoveTo().value() - info.getRobot()->get()->getPos()).angle();
    skillStpInfo.setYaw(skills.current_num() == 0 ? angleToTarget : (info.getPositionToMoveTo().value() - info.getBall()->get()->position).angle());
    skillStpInfo.setDribblerOn(true);

    return skillStpInfo;
}

bool DriveWithBall::isTacticFailing(const StpInfo &info) noexcept { return !info.getRobot().value()->hasBall() || !info.getPositionToMoveTo(); }

bool DriveWithBall::shouldTacticReset(const StpInfo &info) noexcept {
    return skills.current_num() == 1 && info.getRobot()->get()->getYaw().shortestAngleDiff(info.getYaw()) > constants::HAS_BALL_ANGLE;
}

bool DriveWithBall::isEndTactic() noexcept { return false; }

const char *DriveWithBall::getName() { return "Drive With Ball"; }

}  // namespace rtt::ai::stp::tactic
