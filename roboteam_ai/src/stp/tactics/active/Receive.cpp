#include "stp/tactics/active/Receive.h"

#include "stp/constants/ControlConstants.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

Receive::Receive() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> Receive::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    skillStpInfo.setAngle(calculateAngle(info.getRobot().value(), info.getBall().value()));
    skillStpInfo.setDribblerSpeed(100);

    return skillStpInfo;
}

bool Receive::isTacticFailing(const StpInfo &info) noexcept { return !info.getPositionToMoveTo(); }

bool Receive::shouldTacticReset(const StpInfo &info) noexcept {
    double errorMargin = stp::control_constants::GO_TO_POS_ERROR_MARGIN * M_PI;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

bool Receive::isEndTactic() noexcept { return true; }

double Receive::calculateAngle(const world::view::RobotView &robot, const world::view::BallView &ball) { return (ball->position - robot->getPos()).angle(); }

const char *Receive::getName() { return "Receive"; }

}  // namespace rtt::ai::stp::tactic